// simulate the rtlsdr output from the transmitter 
// we are outputing a series of symbols with
// 1. preamble
// 2. (scrambler + convolutional code) as encoding
// 3. (length + data + crc8 + trellis-terminator) as payload

#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <cstring>
#include <algorithm>

#include "decoder/convolutional_encoder.h"
#include "decoder/additive_scrambler.h"
#include "decoder/crc32.h"
#include "decoder/crc8.h"

#include "utility/getopt/getopt.h"
#include "utility/span.h"

#if _WIN32
#include <io.h>
#include <fcntl.h>
#endif

struct IQ_Symbol {
    uint8_t I;
    uint8_t Q;
};

template <typename T>
int push_big_endian_byte(uint8_t* x, T y) {
    auto y_addr = reinterpret_cast<uint8_t*>(&y);
    const int N = sizeof(y) / sizeof(uint8_t);
    for (int i = 0; i < N; i++) {
        x[i] = y_addr[N-1-i];
    }
    return N;
}

// pad preamble bits to be byte aligned
// 2x13-barker codes and 1x2-code and 1x4-code
constexpr uint8_t CONV_POLY[CODE_RATE] = { 0b111, 0b101 };
constexpr uint32_t PREAMBLE_CODE = 0b11111001101011111100110101101101;
constexpr uint16_t SCRAMBLER_CODE = 0b1000010101011001;
constexpr uint32_t CRC32_POLY = 0x04C11DB7;
constexpr uint8_t CRC8_POLY = 0xD5;

auto enc = ConvolutionalEncoder(CONV_POLY);
auto scrambler = AdditiveScrambler(SCRAMBLER_CODE);
auto crc32_calc = CRC32_Calculator(CRC32_POLY);
auto crc8_calc = CRC8_Calculator(CRC8_POLY);

void usage() {
    fprintf(stderr, 
        "simulate_transmitter, a simulator for a 4PSK or 16QAM signal\n\n"
        "Usage:\t[-D (dump encoded IQ symbols out to stdout)]\n"
        "\t[-h (show usage)]\n"
        "\t[-R (dont run in real time)]\n"
        "\t[-f sample rate (default: 2MHz)]\n"
        "\t[-s symbol rate (default: 50kHz)]\n"
        "\t[-b block size (default: 4096)]\n"
        "\t[-t recording time (default: 1s)]\n"
        "\t[-m modulation type (default: 16qam)]\n"
        "\t    options: [16QAM, 4QAM]\n"
    );
}

void create_frame(tcb::span<const uint8_t> input_data, tcb::span<uint8_t> output_data);
std::vector<uint8_t> create_test_data();
std::vector<uint8_t> create_audio_data();
int create_16QAM_symbols(uint8_t x, tcb::span<IQ_Symbol> syms);
int create_4QAM_symbols(uint8_t x, tcb::span<IQ_Symbol> syms);

template <typename T>
void extend_vector(std::vector<T>& dst, std::vector<T>&& src) {
    const size_t final_size = dst.size() + src.size();
    const size_t old_size = dst.size();
    dst.resize(final_size);

    auto write_buffer = tcb::span(dst).last(src.size());
    std::copy_n(src.begin(), src.size(), write_buffer.begin());
}

int main(int argc, char** argv) {
    // check if argument was passed in to dump a byte stream
    int opt;
    bool is_real_time = true;
    bool is_dumping = false;
    float Fs = 2e6;
    float Fsym = 50e3;
    int block_size = 4096;
    float recording_time = 1.0f;

    enum ModulationType { QAM16, QPSK };
    ModulationType modulation_type = ModulationType::QAM16;

    while ((opt = getopt_custom(argc, argv, "s:b:t:m:f:DhR")) != -1) {
        switch (opt) {
        case 'D':
            is_dumping = true;
            break;
        case 'h':
            usage();
            return 0;
        case 'R':
            is_real_time = false;
            break;
        case 'f':
            Fs = static_cast<float>(atof(optarg));
            break;
        case 's':
            Fsym = static_cast<float>(atof(optarg));
            break;
        case 'b':
            block_size = static_cast<int>(atof(optarg));
            break;
        case 't':
            recording_time = static_cast<float>(atof(optarg));
            break;
        case 'm':
            if (strncmp(optarg, "16qam", 5) == 0) {
                modulation_type = ModulationType::QAM16;
            } else if (strncmp(optarg, "4qam", 4) == 0) {
                modulation_type = ModulationType::QPSK;
            } else if (strncmp(optarg, "4psk", 4) == 0) {
                modulation_type = ModulationType::QPSK;
            } else {
                fprintf(stderr, "Unknown modulation type: %s\n", optarg);
                return 1;
            }
            break;
        case '?':
            usage();
            return 0;
        }
    }
    
#if _WIN32
    // NOTE: Windows does extra translation stuff that messes up the file if this isn't done
    // https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/setmode?view=msvc-170
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    
    auto test_data = std::vector<uint8_t>();
    extend_vector(test_data, create_test_data());
    extend_vector(test_data, create_audio_data());
    fprintf(stderr, "Generated %zu bytes\n", test_data.size());

    if (is_dumping) {
        printf("{");
        for (int i = 0; i < test_data.size(); i++) {
            printf("0x%02X,", test_data[i]);
        }
        printf("};\n");
        return 0;
    }

    const float F_block = Fs/(float)(block_size);
    const float T_block = 1.0f/F_block;
    const int N_blocks = static_cast<int>(std::ceil(recording_time/T_block));
    int curr_block = 0;
    
    // number of samples per symbol
    const int Nsamples = (int)std::round(Fs/Fsym);
    const int T_block_microseconds = static_cast<int>(std::ceil(T_block * 1e6));
    
    auto tx_block = std::vector<uint8_t>(block_size*2);
    int curr_tx_block_idx = 0;
    IQ_Symbol sym_out[8];
    // convert to symbols
    auto dt_prev_tx = std::chrono::high_resolution_clock::now();
    while (1) {
        for (int i = 0; i < test_data.size(); i++) {
            const auto x = test_data[i];
            int total_symbols = 0;
            switch (modulation_type) {
            case ModulationType::QAM16:
                total_symbols = create_16QAM_symbols(x, sym_out);
                break;
            case ModulationType::QPSK:
            default:
                total_symbols = create_4QAM_symbols(x, sym_out);
                break;
            }

            for (int j = 0; j < total_symbols; j++) {
                auto sym = sym_out[j];

                for (int k = 0; k < Nsamples; k++) {
                    tx_block[curr_tx_block_idx  ] = sym.I;
                    tx_block[curr_tx_block_idx+1] = sym.Q;
                    curr_tx_block_idx += 2;

                    // transmit the block
                    if (curr_tx_block_idx >= tx_block.size()) {
                        const size_t nb_write = fwrite(tx_block.data(), sizeof(uint8_t), tx_block.size(), stdout);
                        if (nb_write != tx_block.size()) {
                            fprintf(stderr, "Failed to write symbol %zu/%zu\n", nb_write, tx_block.size());
                            return 0;
                        }
                        curr_tx_block_idx = 0;
                        curr_block++;

                        // if it is real time, add a delay
                        if (is_real_time) {
                            auto dt_curr_tx = std::chrono::high_resolution_clock::now();
                            const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(dt_curr_tx-dt_prev_tx);
                            int T_offset =  static_cast<int>(dt.count());
                            int delay = T_offset - T_block_microseconds;
                            dt_prev_tx = dt_curr_tx;
                            std::this_thread::sleep_for(std::chrono::microseconds((T_block_microseconds - delay)/2));
                        // if this isn't real time, we will impose a time recording limit
                        } else {
                            if (curr_block >= N_blocks) {
                                return 0;
                            }
                        }
                   }
                }
            }
        }
    }
    return 0;
}

int create_16QAM_symbols(uint8_t x, tcb::span<IQ_Symbol> syms) {
    static const uint8_t _gray_code[4] = {0b00, 0b01, 0b11, 0b10};

    uint8_t I1 = (x & 0b11000000) >> 6;
    uint8_t Q1 = (x & 0b00110000) >> 4;
    uint8_t I2 = (x & 0b00001100) >> 2;
    uint8_t Q2 = (x & 0b00000011);

    const int A = 64;
    const int B = 128;
    // Average of QAM signal is 1.5 = (0+1+2+3)/4 = 6/4 = 3/2
    const int DC = (uint8_t)((float)A * 1.5f);

    static auto get_val = [A,B,DC](uint8_t v) {
        int x = (int)_gray_code[v];
        x = x*A + B - DC;
        return (uint8_t)x;        
    };

    I1 = get_val(I1);
    Q1 = get_val(Q1);
    I2 = get_val(I2);
    Q2 = get_val(Q2);

    syms[0].I = I1;
    syms[0].Q = Q1;
    syms[1].I = I2;
    syms[1].Q = Q2;

    return 2;
}

int create_4QAM_symbols(uint8_t x, tcb::span<IQ_Symbol> syms) {
    // NOTE: 4QAM is already gray code by itself
    const uint8_t A = 64;
    const uint8_t B = 128;
    // Average amplitude is 0.5 = (0+1)/2 = 0.5
    const uint8_t DC = A/2;

    static auto get_val = [A,B,DC](uint8_t v) {
        const int x = (int)v*A + B - DC;
        return (uint8_t)x;        
    };

    int j = 0;
    for (int i = 0; i < 8; i+=2) {
        const int shift = 7-i;
        uint8_t I = (x >> shift    ) & 0x1;
        uint8_t Q = (x >> (shift-1)) & 0x1;
        syms[j].I = get_val(I);
        syms[j].Q = get_val(Q);
        j++;
    }
    return 4;
}

void create_frame(tcb::span<const uint8_t> input_data, tcb::span<uint8_t> output_data) {
    // encoding size is given as the following
    // 4: preamble
    // additive scrambler + fec of 1/2 K=3 [7,5] code
    // 2: length of payload
    // N: payload
    // 1: CRC8 
    // 1: NULL trellis terminator

    // T = 4 + 2*(2+N+1+1)
    // T = 2N + 12
    const size_t expected_output_size = 2*input_data.size() + 12;
    assert(expected_output_size == output_data.size());

    const size_t Nx = input_data.size();
    const size_t Ny = output_data.size();
    auto x = input_data;
    auto y = output_data;

    int offset = 0;
    offset += push_big_endian_byte(&y[offset], PREAMBLE_CODE);

    const int scrambler_offset = offset;

    enc.reset();
    scrambler.reset();

    uint16_t Nx_copy = static_cast<uint16_t>(Nx);
    auto Nx_addr = reinterpret_cast<uint8_t*>(&Nx_copy);
    for (int i = 0; i < sizeof(Nx_copy); i++) {
        auto enc_out = enc.consume_byte(Nx_addr[i]);
        offset += push_big_endian_byte(&y[offset], enc_out);
    }

    for (int i = 0; i < Nx; i++) {
        auto enc_out = enc.consume_byte(x[i]);
        offset += push_big_endian_byte(&y[offset], enc_out);
    }

    uint8_t crc8 = crc8_calc.process(x.data(), int(x.size()));
    auto crc8_addr = reinterpret_cast<uint8_t*>(&crc8);
    for (int i = 0; i < sizeof(crc8); i++) {
        auto enc_out = enc.consume_byte(crc8_addr[i]);
        offset += push_big_endian_byte(&y[offset], enc_out);
    }

    {
        uint8_t trellis_terminator = 0x00;
        auto enc_out = enc.consume_byte(trellis_terminator);
        offset += push_big_endian_byte(&y[offset], enc_out);
    }

    for (int i = scrambler_offset; i < offset; i++) {
        y[i] = scrambler.process(y[i]);
    }
}

std::vector<uint8_t> create_test_data() {
    constexpr size_t TOTAL_STRINGS = 4;
    const char* data[TOTAL_STRINGS] = {
        "Test string 1",
        "Test string 2",
        "Trust in the LORD with all your heart, and do not lean on your own understanding. In all your ways acknowledge Him, and He will make straight your paths. May the God of hope fill you with all joy and peace as you trust in Him, so that you may overflow with hope by the power of the Holy Spirit.",
        "Test string 4",
    };

    // encoding size is given as the following
    // 4: preamble
    // additive scrambler + fec of 1/2 K=3 [7,5] code
    // 2: length of payload
    // N: payload
    // 1: CRC8 
    // 1: Trellis terminator 

    // T = 4 + 2*(2+N+1)
    // T = 2N + 12

    size_t string_lengths[TOTAL_STRINGS] = {0};
    size_t encoded_lengths[TOTAL_STRINGS] = {0};
    size_t total_encoded_length = 0;
    for (size_t i = 0; i < TOTAL_STRINGS; i++) {
        const char* input_data = data[i];
        const size_t input_length = strlen(input_data);
        const size_t encoded_length = 2*input_length + 12;

        string_lengths[i] = input_length;
        encoded_lengths[i] = encoded_length;
        total_encoded_length += encoded_length;
    }

    auto frame = std::vector<uint8_t>(total_encoded_length);
    auto write_buffer = tcb::span(frame);
    for (size_t i = 0; i < TOTAL_STRINGS; i++) {
        const auto input_data = tcb::span(reinterpret_cast<const uint8_t*>(data[i]), string_lengths[i]);
        const size_t encoded_length = encoded_lengths[i];
        create_frame(input_data, write_buffer.first(encoded_length));
        write_buffer = write_buffer.subspan(encoded_length);
    }

    return frame;
}

std::vector<uint8_t> create_audio_data() {
    // NOTE: prototype has this value set to 100 to delineate audio packets which is an ugly hack!!!
    const size_t AUDIO_PACKET_BLOCK_SIZE = 100;
    const size_t total_encoded_per_block = 2*AUDIO_PACKET_BLOCK_SIZE + 12;
    const size_t total_audio_blocks = 1000;
    const size_t total_encoded = total_encoded_per_block * total_audio_blocks;
    
    auto block = std::vector<uint8_t>(AUDIO_PACKET_BLOCK_SIZE);
    auto output_frame = std::vector<uint8_t>(total_encoded);
    auto write_buffer = tcb::span(output_frame); 
    float dt = 0.0f;
    for (size_t curr_block = 0; curr_block < total_audio_blocks; curr_block++) {
        // generate sine wave
        for (size_t i = 0; i < AUDIO_PACKET_BLOCK_SIZE; i++) {
            const uint8_t sample = uint8_t(std::sin(dt)*126.0 + 127.0);
            block[i] = sample;
            dt += 0.1f;
        }
        // encode block
        create_frame(block, write_buffer.first(total_encoded_per_block));
        write_buffer = write_buffer.subspan(total_encoded_per_block);
    }

    return output_frame;
}
