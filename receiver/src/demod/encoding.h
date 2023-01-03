#pragma once

#include <assert.h>
#include <stdint.h>

// lookup table for xor bit sum
static uint8_t XOR_SUM_LUT[32] = {0,1,1,0,1,0,0,1, 1,0,0,1,0,1,1,0, 1,0,0,1,0,1,1,0, 0,1,1,0,1,0,0,1};
static int BIT_COUNT_LUT[32] = {0,1,1,2,1,2,2,3, 1,2,2,3,2,3,3,4, 1,2,2,3,2,3,3,4, 2,3,3,4,3,4,4,5};

// typedef uint32_t encoder_decoder_type;
// constexpr uint8_t G[4] = {0b1000, 0b0100, 0b0010, 0b0001};
// constexpr int G_REG_SIZE = 4;
// constexpr uint8_t G[4] = {0b111, 0b101, 0b110, 0b001};
// constexpr int G_REG_SIZE = 3;
// constexpr uint8_t G[4] = {0b11, 0b00, 0b10, 0b01};
// constexpr int G_REG_SIZE = 2;

typedef uint16_t encoder_decoder_type;
constexpr uint8_t G[2] = {0b111, 0b101};
constexpr int G_REG_SIZE = 3;

constexpr int G_SYM_SIZE = sizeof(G) / sizeof(uint8_t);
constexpr uint8_t G_REG_MASK = (1u<<G_REG_SIZE) -1;
constexpr uint8_t G_SYM_MASK = (1u<<G_SYM_SIZE) -1;

template <typename T>
class ConvolutionalEncoder {
private:
    uint16_t reg = 0;
    const uint16_t reg_mask = G_REG_MASK << 8;
public:
    // reset registers for each new block
    void reset(void) {
        reg = 0;
    }
    T consume_byte(uint8_t x) {
        assert(sizeof(T) == G_SYM_SIZE);
        reg = reg | x;
        T y = 0;
        for (int y_offset = G_SYM_SIZE*8 - 1; y_offset >= 0; y_offset-=G_SYM_SIZE) {
            reg = reg << 1;
            uint8_t v = (reg & reg_mask) >> 8;
            for (int i  = 0; i < G_SYM_SIZE; i++) {
                y |= (XOR_SUM_LUT[v & G[i]] << (y_offset-i));
            }
        }

        return y;
    }
};

template <typename T>
class ViterbiDecoder {
public:
    struct Transition {
        uint8_t prev_state: 7;
        uint8_t input: 1;
        int total_error;
    };
public:
    // traceback length
    const int nb_traceback;
    const int nb_states = 1<<G_REG_SIZE;
    const int nb_transition_list_entries;
    // nb_transition_list_entries = nb_states * nb_traceback
    // index = curr_state*nb_states + state_offset
    Transition* transition_list;
    int curr_step = 0;
    // index = (state << 1) | input
    uint8_t symbol_output_lut[1<<(G_REG_SIZE+1)];
    // flush traceback if we have reached the limit
    int nb_paths_stored = 0;
public:
    ViterbiDecoder(const int _N)
    : nb_traceback(_N), nb_transition_list_entries(_N*nb_states)
    {
        // generate lookup table for output symbols
        for (uint8_t i = 0; i < nb_states*2; i++) {
            uint8_t reg = i & G_REG_MASK;
            uint8_t y = 0;
            for (int i  = 0; i < G_SYM_SIZE; i++) {
                y |= (XOR_SUM_LUT[reg & G[i]] << (G_SYM_SIZE-1-i));
            }
            symbol_output_lut[i] = y;
        }

        transition_list = new Transition[nb_transition_list_entries];
        reset();
    }
    ~ViterbiDecoder() {
        delete [] transition_list;
    }
    void reset() {
        nb_paths_stored = 0;

        for (int i  = 0; i < nb_transition_list_entries; i++) {
            auto& e = transition_list[i];
            e.input = 0;
            e.prev_state = 0;
            e.total_error = INT32_MAX/4;
        }

        // our initial starting state is 0
        curr_step = 0;
        auto index = get_index(0, 0);
        auto& e = transition_list[index];
        e.total_error = 0;
    }

    // rv = number of bytes decoded
    int process(uint8_t* x, const int N, uint8_t* y, const int Ny, bool is_flush=true) {
        assert((N % G_SYM_SIZE) == 0);
        assert(Ny >= (N/G_SYM_SIZE));

        int curr_y_bit = 0;

        for (int z = 0; z < N; z+=G_SYM_SIZE) {
            T sym_group = 0;
            // fix endianess 
            for (int i = 0; i < G_SYM_SIZE; i++) {
                sym_group |= x[z+i] << ((G_SYM_SIZE-1-i)*8);
            }

            for (int j = 0; j < 8; j++) {
                const int shift = G_SYM_SIZE*(7-j);
                const uint8_t sym = (sym_group & (G_SYM_MASK << shift)) >> shift;
                process_symbol(sym);
                nb_paths_stored++;

                if (nb_paths_stored < nb_traceback) {
                    continue;
                }
                perform_traceback(y, curr_y_bit);
            }
        }

        // get remaining bits if we are flushing it all
        if (is_flush) {
            flush_tracebacks(y, curr_y_bit);
        }

        // NOTE: For this to be true with flush=false, the traceback length must be of a particular value
        // Number of bits not outputed are traceback_length-1
        // Therefore traceback_length=8*N+1
        assert((curr_y_bit % 8) == 0);
        return curr_y_bit/8;
    }

    int get_curr_error(void) {
        int lowest_error = INT32_MAX;
        for (uint8_t i = 0; i < nb_states; i++) {
            const auto index = get_index(curr_step, i);
            auto& e = transition_list[index];
            if (e.total_error < lowest_error) {
                lowest_error = e.total_error;
            }
        }
        return lowest_error;
    }

private:
    // each input byte to encoder must produce at least 2 bytes
    // our decoder is just the inverse of this
    void process_symbol(uint8_t sym) {
        assert(sym <= G_SYM_MASK);
        const int next_step = (curr_step+1) % nb_traceback;

        for (int s = 0; s < nb_states; s++) {
            const auto idx = get_index(next_step, s);
            transition_list[idx].total_error = INT32_MAX;
        }

        // for each old state get hamming distandce error of transition
        // keep the ones with lowest hamming distance 
        for (uint8_t s = 0; s < 2*nb_states; s++) {
            const uint8_t curr_state = (s & (G_REG_MASK << 1)) >> 1;
            const uint8_t next_state = (s & G_REG_MASK);
            const uint8_t input = s & 0b1;

            assert(curr_state <= (1 << G_REG_SIZE));
            assert(next_state <= (1 << G_REG_SIZE));

            const auto prev_idx = get_index(curr_step, static_cast<int>(curr_state));
            const auto next_idx = get_index(next_step, static_cast<int>(next_state));

            auto& prev_e = transition_list[prev_idx];
            auto& next_e = transition_list[next_idx];

            const auto pred_sym = symbol_output_lut[s];
            const auto error = BIT_COUNT_LUT[pred_sym ^ sym];

            const auto acc_error = prev_e.total_error + error;
            if (acc_error < next_e.total_error) {
                next_e.total_error = acc_error;
                next_e.prev_state = curr_state;
                next_e.input = input;
            }
        }

        curr_step = next_step;
    }

    uint8_t find_best_state() {
        // find our initial lowest error path
        int lowest_error = INT32_MAX;
        uint8_t best_state = 0;
        for (uint8_t i = 0; i < nb_states; i++) {
            const auto index = get_index(curr_step, i);
            auto& e = transition_list[index];
            if (e.total_error < lowest_error) {
                lowest_error = e.total_error;
                best_state = i;
            }
        }

        return best_state;
    }

    void flush_tracebacks(uint8_t* y, int& curr_y_bit) {
        // perform traceback
        // uint8_t curr_state = find_best_state();
        // with trellis termination, we always start at state 0 when flushing
        uint8_t curr_state = 0;
        for (int i = 0; i < nb_paths_stored; i++) {
            const auto step = (nb_traceback+curr_step-i) % nb_traceback;
            const auto index = get_index(step, curr_state);
            auto& e = transition_list[index];

            // write the bit
            const int write_bit_index = curr_y_bit+nb_paths_stored-1-i;
            const int write_bit_offset = write_bit_index % 8;
            const int write_byte_index = (write_bit_index - write_bit_offset) / 8;
            y[write_byte_index] |= (e.input << (7 - write_bit_offset));

            curr_state = e.prev_state;
        }

        curr_y_bit += nb_paths_stored;
        nb_paths_stored = 0;
    }

    void perform_traceback(uint8_t* y, int& curr_y_bit) {
        // perform traceback
        uint8_t curr_state = find_best_state();
        uint8_t input = 0;
        for (int i = 0; i < nb_paths_stored; i++) {
            const auto step = (nb_traceback+curr_step-i) % nb_traceback;
            const auto index = get_index(step, curr_state);
            auto& e = transition_list[index];

            curr_state = e.prev_state;
            input = e.input;
        }

        // write the bit
        const int write_bit_index = curr_y_bit;
        const int write_bit_offset = write_bit_index % 8;
        const int write_byte_index = (write_bit_index - write_bit_offset) / 8;
        y[write_byte_index] |= (input << (7 - write_bit_offset));

        curr_y_bit += 1;
        nb_paths_stored--;
    }

    inline int get_index(int step, int state) {
        assert(state < nb_states);
        const auto i = step*nb_states + state;
        assert(i <= nb_transition_list_entries);
        return i;
    }
};