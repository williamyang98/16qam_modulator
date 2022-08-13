// simulate the rtlsdr output from the transmitter 
// we are outputing a series of symbols with
// 1. preamble
// 2. (scrambler + convolutional code) as encoding
// 3. (data + crc32) as payload

// program outputs blocks at a rate of Fs/block_size
// Fs = 250kHz
// block_size = 4096

// symbol rate is 10kHz
// each symbol has Fs/Fsym samples
// Nsym = 25

#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#include <cmath>
#include <chrono>
#include <thread>

#include "encoding.h"
#include "additive_scrambler.h"
#include "util.h"
#include "crc32.h"
#include "crc8.h"

// pad preamble bits to be byte aligned
// 2x13-barker codes and 1x2-code and 1x4-code
constexpr uint32_t PREAMBLE_CODE = 0b11111001101011111100110101101101;

constexpr uint16_t SCRAMBLER_CODE = 0b1000010101011001;
constexpr uint32_t CRC32_POLY = 0x04C11DB7;
constexpr uint8_t CRC8_POLY = 0xD5;

int main(int argc, char** argv) {
    assert(sizeof(encoder_decoder_type) == G_SYM_SIZE);
    auto enc = ConvolutionalEncoder<encoder_decoder_type>();
    auto scrambler = AdditiveScrambler(SCRAMBLER_CODE);
    auto crc32_calc = CRC32_Calculator(CRC32_POLY);
    auto crc8_calc = CRC32_Calculator(CRC8_POLY);

    // perform our packet framing and fec
    auto create_frame = 
        [&enc, &scrambler, &crc8_calc]
        (uint8_t* x, const int Nx, uint8_t* y, const int Ny) 
    {
        int offset = 0;
        offset += push_big_endian_byte(&y[offset], PREAMBLE_CODE);


        enc.reset();
        scrambler.reset();

        uint16_t Nx_copy = static_cast<uint16_t>(Nx);
        auto Nx_addr = reinterpret_cast<uint8_t*>(&Nx_copy);
        for (int i = 0; i < sizeof(Nx_copy); i++) {
            auto enc_out = enc.consume_byte(Nx_addr[i]);
            auto scrambler_out = scrambler.process(enc_out);
            offset += push_big_endian_byte(&y[offset], scrambler_out);
        }

        for (int i = 0; i < Nx; i++) {
            auto enc_out = enc.consume_byte(x[i]);
            auto scrambler_out = scrambler.process(enc_out);
            offset += push_big_endian_byte(&y[offset], scrambler_out);
        }

        // auto crc32 = crc32_calc.process(x, Nx);
        // auto crc32_addr = reinterpret_cast<uint8_t*>(&crc32);

        auto crc8 = crc8_calc.process(x, Nx);
        auto crc8_addr = reinterpret_cast<uint8_t*>(&crc8);

        for (int i = 0; i < sizeof(crc8); i++) {
            auto enc_out = enc.consume_byte(crc8_addr[i]);
            auto scrambler_out = scrambler.process(enc_out);
            offset += push_big_endian_byte(&y[offset], scrambler_out);
        }

        return offset;
    };

    uint8_t frame_out[4096] = {0};
    const int frame_size = sizeof(frame_out) / sizeof(uint8_t);

    uint8_t data[] = "Hello motherfucker! Hey it is me Siraj and I want to scam you xD! #@";
    const int data_size = sizeof(data) / sizeof(uint8_t);

    uint8_t data2[] = "I am a turnip and I want to stick a nuke up your ass";
    const int data2_size = sizeof(data2) / sizeof(uint8_t);

    uint8_t data3[] = "Trust in the LORD with all your heart, and do not lean on your own understanding. In all your ways acknowledge Him, and He will make straight your paths. May the God of hope fill you with all joy and peace as you trust in Him, so that you may overflow with hope by the power of the Holy Spirit.";
    const int data3_size = sizeof(data3) / sizeof(uint8_t);

    uint8_t data4[] = "Small verse";
    const int data4_size = sizeof(data4) / sizeof(uint8_t);

    int total_written = 0;
    total_written += create_frame(data4, data4_size, &frame_out[total_written], frame_size-total_written);
    total_written += create_frame(data3, data3_size, &frame_out[total_written], frame_size-total_written);
    total_written += create_frame(data2, data2_size, &frame_out[total_written], frame_size-total_written);
    total_written += create_frame(data, data_size, &frame_out[total_written], frame_size-total_written);

    // check if argument was passed in to dump a byte stream
    if ((argc >= 2) && (strncmp(argv[1], "--dump", 6) == 0)) {
        printf("{");
        for (int i = 0; i < total_written; i++) {
            printf("0x%02X,", frame_out[i]);
        }
        printf("};\n");
        return 0;
    }

    // We are simulating a 250kHz sampling rate
    // Symbol rate is 10kHz
    // This means each symbol gets 25 samples

    // We are transmitting in blocks of 4096 samples
    // This gives a block output frequency of 61Hz

    const float Fs = 1e6;
    const float Fsym = 50e3;
    const int block_size = 4096;
    const float F_block = Fs/(float)(block_size);
    const float T_block = 1.0f/F_block;

    const int Nsamples = (int)std::floorf(Fs/Fsym);

    const int T_block_microseconds = static_cast<int>(std::ceil(T_block * 1e6));

    uint8_t tx_block[block_size*2] = {0};
    int curr_tx_block_idx = 0;

    // convert to symbols
    auto dt_prev_tx = std::chrono::high_resolution_clock::now();
    while (1) {
        for (int i = 0; i < total_written; i++) {
            const auto x = frame_out[i];
            for (int j = 0; j < 8; j+=2) {
                const int shift = 7-j;
                uint8_t I = (x & (1 << shift)) >> shift;
                uint8_t Q = (x & (1 << (shift-1))) >> (shift-1);
                I = (I*8) + 128;
                Q = (Q*8) + 128;

                for (int k = 0; k < Nsamples; k++) {
                    tx_block[curr_tx_block_idx  ] = I;
                    tx_block[curr_tx_block_idx+1] = Q;
                    curr_tx_block_idx += 2;

                    // transmit the block
                    if (curr_tx_block_idx >= (block_size*2)) {
                        fwrite(tx_block, 2, block_size, stdout);
                        curr_tx_block_idx = 0;

                        auto dt_curr_tx = std::chrono::high_resolution_clock::now();
                        const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(dt_curr_tx-dt_prev_tx);
                        int T_offset =  static_cast<int>(dt.count());
                        dt_prev_tx = dt_curr_tx;
                        std::this_thread::sleep_for(std::chrono::microseconds(T_block_microseconds - T_offset));
                        // std::this_thread::sleep_for(std::chrono::microseconds(10));
                    }
                }
            }
        }
    }

    return 0;
}