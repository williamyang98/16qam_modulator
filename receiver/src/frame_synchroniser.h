#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <assert.h>

#include "crc8.h"
#include "encoding.h"
#include "additive_scrambler.h"

// shift register which searches for preamble sequence
template <typename T>
class PreambleFilter {
private:
    T reg;
    const T preamble;
public:
    PreambleFilter(const T _preamble) 
    : preamble(_preamble) {
        reg = 0;
    }
    void reset() {
        reg = 0;
    }
    uint8_t process(const uint8_t sym, const int N=2) {
        const uint8_t mask = (1<<N)-1;
        reg = reg << N;
        reg = reg | (sym & mask);

        return (reg ^ preamble) == 0;
    }
};

template <typename T>
class FrameSynchroniser {
public:
    // Payload state for different process results
    // BLOCK_SIZE_OK, BLOCK_SIZE_ERR: length is defined, but buf is NULL
    // PAYLOAD_OK, PAYLOAD_ERR: buf is defined, and crc8 and decoding error is updated
    enum ProcessResult { NONE, PREAMBLE_FOUND, BLOCK_SIZE_OK, BLOCK_SIZE_ERR, PAYLOAD_OK, PAYLOAD_ERR };
    struct Payload {
        uint16_t length;
        uint8_t* buf;
        uint8_t crc8_received;
        uint8_t crc8_calculated;
        bool crc8_mismatch;
        int decoded_error;
        void reset() {
            length = 0;
            buf = NULL;
            crc8_received = 0;
            crc8_calculated = 0;
            crc8_mismatch = false;
            decoded_error = -1;
        }
    };
    // updates after each preamble detection
    struct PreambleState {
        int selected_phase;
        bool phase_conflict;
        int desync_bitcount;
    };
private:
    enum State { WAIT_PREAMBLE, WAIT_BLOCK_SIZE, WAIT_PAYLOAD };
private:
    PreambleFilter<T> preamble_filters[4];
    std::complex<float> preamble_mixers[4];
    AdditiveScrambler descrambler;
    ViterbiDecoder<encoder_decoder_type> vitdec;
    CRC8_Calculator crc8_calc;
    // internal buffers for decoding
    const int nb_buffer;
    uint8_t* descramble_buffer;
    uint8_t* encoded_buffer;
    uint8_t* decoded_buffer;
    // keep track of position in buffers
    int encoded_bits = 0;
    int encoded_bytes = 0;
    int decoded_bytes = 0;
    // keep track of encoded and decoded block size
    const int nb_bytes_for_block_size = 16; // minimum required bytes to decipher block size
    int decoded_block_size = 0;
    int encoded_block_size = 0; 
    State state;
    int bits_since_preamble = 0;
public:
    Payload payload;
    PreambleState preamble_state;
public:
    FrameSynchroniser(const uint32_t _preamble, const uint16_t _scrambler_code, const uint8_t _crc8_poly, const int _buffer_size) 
    :   preamble_filters{_preamble, _preamble, _preamble, _preamble},
        descrambler(_scrambler_code),
        vitdec(25),
        crc8_calc(_crc8_poly),
        nb_buffer(_buffer_size)
    {
        // generate our quadrature phase shifts 
        for (int i = 0; i < 4; i++) {
            const float PI_2 = 3.1415f/2.0f;
            preamble_mixers[i] = std::complex<float>(std::cosf(i*PI_2), std::sinf(i*PI_2));
        }
        state = State::WAIT_PREAMBLE;

        // the descramble and encoded buffers are the same size
        // the descrambler is of rate 1/1
        descramble_buffer = new uint8_t[nb_buffer];
        encoded_buffer = new uint8_t[nb_buffer];
        // the decoded buffer is always smaller than the encoded frame
        decoded_buffer = new uint8_t[nb_buffer];

        preamble_state.desync_bitcount = 0;
        preamble_state.phase_conflict = false;
        preamble_state.selected_phase = 0;
    }

    ~FrameSynchroniser() {
        delete [] descramble_buffer;
        delete [] encoded_buffer;
        delete [] decoded_buffer;
    }

    ProcessResult process(const std::complex<float> IQ) {
        switch (state) {
        case State::WAIT_PREAMBLE:
            return process_await_preamble(IQ); 
        case State::WAIT_BLOCK_SIZE:
            return process_await_block_size(IQ);
        case State::WAIT_PAYLOAD:
            return process_await_payload(IQ);;
        default:
            return ProcessResult::NONE;
        }
    }

    State get_state() {
        return state;
    }
private:
    ProcessResult process_await_preamble(const std::complex<float> IQ) {
        int total_preambles_found = 0;

        bits_since_preamble += 2;
        for (int i = 0; i < 4; i++) {
            auto IQ_phi = IQ * preamble_mixers[i];
            uint8_t sym  = 0;
            sym |= (IQ_phi.real() > 0.0f) ? 0b10 : 0b00;
            sym |= (IQ_phi.imag() > 0.0f) ? 0b01 : 0b00;

            const bool res = preamble_filters[i].process(sym);
            if (!res) {
                continue;
            }

            preamble_state.selected_phase = i;
            total_preambles_found += 1;
            preamble_state.desync_bitcount = bits_since_preamble-32;
        }

        if (total_preambles_found > 0) {
            preamble_state.phase_conflict = (total_preambles_found > 1);
            state = State::WAIT_BLOCK_SIZE;
            reset_decoders();
            payload.reset();
            return ProcessResult::PREAMBLE_FOUND;
        }

        return ProcessResult::NONE;
    }

    ProcessResult process_await_block_size(const std::complex<float> IQ) {
        process_decoder_symbol(IQ);
        bool is_done = 
            (encoded_bytes == nb_bytes_for_block_size) &&
            (encoded_bits == 0);
        
        if (!is_done) {
            return ProcessResult::NONE;
        }

        decoded_bytes += vitdec.process(
            &encoded_buffer[0], 
            nb_bytes_for_block_size, 
            &decoded_buffer[0], 
            nb_buffer-decoded_bytes,
            false);

        const uint16_t rx_block_size = *reinterpret_cast<uint16_t*>(&decoded_buffer[0]);
        payload.length = rx_block_size;
        // our receiving block size must fit into the buffer with length and crc
        constexpr int frame_overhead = 2+1; // 2byte length and crc8
        if (rx_block_size > (nb_buffer-frame_overhead)) {
            payload.buf = NULL; 
            bits_since_preamble = 0;
            state = State::WAIT_PREAMBLE; 
            return ProcessResult::BLOCK_SIZE_ERR;
        } else {
            payload.buf = NULL;
            decoded_block_size = rx_block_size;
            encoded_block_size = 2*(decoded_block_size+2+1);
            state = State::WAIT_PAYLOAD; 
            return ProcessResult::BLOCK_SIZE_OK;
        }
    }

    ProcessResult process_await_payload(const std::complex<float> IQ) {
        process_decoder_symbol(IQ);
        bool is_done = 
            (encoded_bytes == encoded_block_size) &&
            (encoded_bits == 0);

        if (!is_done) {
            return ProcessResult::NONE;
        }

        decoded_bytes += vitdec.process(
            &encoded_buffer[nb_bytes_for_block_size], 
            encoded_block_size-nb_bytes_for_block_size, 
            &decoded_buffer[decoded_bytes], 
            nb_buffer-decoded_bytes,
            true);

        constexpr int frame_length_field_size = 2;
        uint8_t* payload_buf = &decoded_buffer[frame_length_field_size];

        const uint8_t crc8_true = decoded_buffer[decoded_bytes-1];
        const uint8_t crc8_pred = crc8_calc.process(payload_buf, decoded_block_size);
        const auto dist_err = vitdec.get_curr_error();
        const bool crc8_mismatch = (crc8_true != crc8_pred);

        state = State::WAIT_PREAMBLE;
        bits_since_preamble = 0;

        payload.buf = payload_buf;
        payload.crc8_calculated = crc8_pred;
        payload.crc8_received = crc8_true;
        payload.crc8_mismatch = crc8_mismatch;
        payload.decoded_error = dist_err;

        if (crc8_mismatch) {
            return ProcessResult::PAYLOAD_ERR;
        } else {
            return ProcessResult::PAYLOAD_OK;
        }
    }

    void process_decoder_symbol(const std::complex<float>IQ) {
        const auto IQ_phi = IQ * preamble_mixers[preamble_state.selected_phase];
        uint8_t sym  = 0;
        sym |= (IQ_phi.real() > 0.0f) ? 0b10 : 0b00;
        sym |= (IQ_phi.imag() > 0.0f) ? 0b01 : 0b00;

        if (encoded_bits == 0) {
            descramble_buffer[encoded_bytes] = 0;
        }

        descramble_buffer[encoded_bytes] |= (sym << (6-encoded_bits));
        encoded_bits += 2;

        if (encoded_bits == 8) {
            encoded_bits = 0;
            encoded_buffer[encoded_bytes] = descrambler.process(descramble_buffer[encoded_bytes]);
            encoded_bytes += 1;
        }
    }

    void reset_decoders() {
        // we need to clear this since the viterbi decoder ORs in the bits
        for (int i = 0; i < decoded_bytes; i++) {
            decoded_buffer[i] = 0;
        }

        encoded_bits = 0;
        encoded_bytes = 0;
        decoded_bytes = 0;

        decoded_block_size = 0;
        encoded_block_size = 0;

        descrambler.reset();
        vitdec.reset();
    }
};