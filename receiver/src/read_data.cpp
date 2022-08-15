#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <assert.h>

#include "carrier_dsp.h"
#include "frame_synchroniser.h"

#define PRINT_LOG 0

#if PRINT_LOG 
  #define LOG_MESSAGE(...) fprintf(stderr, ##__VA_ARGS__)
#else
  #define LOG_MESSAGE(...) (void)0
#endif

constexpr uint32_t PREAMBLE_CODE = 0b11111001101011111100110101101101;
constexpr uint16_t SCRAMBLER_CODE = 0b1000010101011001;
constexpr uint32_t CRC32_POLY = 0x04C11DB7;
constexpr uint32_t CRC8_POLY = 0xD5;

int main(int argc, char **argv) {
    FILE* fp_in = stdin;
    if (argc > 1) {
        FILE* tmp = NULL;
        fopen_s(&tmp, argv[1], "r");
        if (tmp == NULL) {
            LOG_MESSAGE("Failed to open file for reading\n");
            return 1;
        } 
        fp_in = tmp;
    }

    // carrier demodulator
    const int block_size = 4096;
    CarrierToSymbolDemodulator demod(block_size);
    auto x_buffer = new std::complex<uint8_t>[block_size];
    auto y_buffer = new std::complex<float>[block_size];

    demod.pll_mixer.fcenter = -1000;

    const int rx_buffer_size = 4096;
    auto frame_sync = FrameSynchroniser<uint32_t>(PREAMBLE_CODE, SCRAMBLER_CODE, CRC8_POLY, rx_buffer_size);

    uint16_t* pcm_buffer = new uint16_t[rx_buffer_size];

    auto payload_handler = [&pcm_buffer](uint8_t* x, const uint16_t N) {
        // if not an audio block
        if (N != 100) {
            LOG_MESSAGE("message=%.*s\n", N, x);
            return;
        }

        for (int i = 0; i < N; i++) {
            const uint8_t v = x[i];
            int16_t v0 = static_cast<int16_t>(v);
            v0 = v0-127;
            v0 = v0 * 16;
            v0 = v0 + (1u << 8);
            uint16_t v1 = (uint16_t)(v0);
            v1 = v1 * 2;
            pcm_buffer[i] = (uint16_t)(v1);
        }
        fwrite(pcm_buffer, sizeof(uint16_t), N, stdout);
    };

    size_t rd_block_size = 0;
    int rd_total_blocks = 0;
    while ((rd_block_size = fread(reinterpret_cast<uint8_t*>(x_buffer), 2*sizeof(uint8_t), block_size, fp_in)) > 0) {
        if (rd_block_size != block_size) {
            LOG_MESSAGE("Got mismatched block size after %d blocks\n", rd_total_blocks);
            break;
        }
        rd_total_blocks++; 

        auto total_symbols = demod.ProcessBlock(x_buffer, y_buffer);
        for (int i = 0; i < total_symbols; i++) {
            const auto IQ = y_buffer[i];
            const auto res = frame_sync.process(IQ);
            using ProcessResult = FrameSynchroniser<uint32_t>::ProcessResult;
            switch (res) {
            case ProcessResult::PREAMBLE_FOUND:
                {
                    auto& state = frame_sync.preamble_state;
                    LOG_MESSAGE("Preamble: phase %d, conflict=%d, desync=%d\n", 
                            state.selected_phase, state.phase_conflict, state.desync_bitcount);
                }
                break;
            case ProcessResult::BLOCK_SIZE_OK:
                LOG_MESSAGE("Got block size: %d\n", frame_sync.payload.length);
                break;
            case ProcessResult::BLOCK_SIZE_ERR:
                LOG_MESSAGE("Got invalid block size: %d\n", frame_sync.payload.length);
                break;
            case ProcessResult::PAYLOAD_ERR:
                LOG_MESSAGE("Got mismatching crc on payload\n");
                break;
            case ProcessResult::PAYLOAD_OK:
                {
                    auto& p = frame_sync.payload;
                    payload_handler(p.buf, p.length);
                }
                break;
            case ProcessResult::NONE:
            default:
                break;
            }
        }

    }

    delete [] x_buffer;
    delete [] y_buffer;
    delete [] pcm_buffer;

    fclose(fp_in);

    LOG_MESSAGE("Exiting\n");

    return 0;
}
