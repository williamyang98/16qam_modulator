#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <assert.h>

#include "carrier_dsp.h"
#include "carrier_demodulator_spec.h"
#include "frame_synchroniser.h"

#include <io.h>
#include <fcntl.h>

#define PRINT_LOG 1

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

    // NOTE: Windows does extra translation stuff that messes up the file if this isn't done
    // https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/setmode?view=msvc-170
    freopen(NULL, "rb", fp_in);
    _setmode(fileno(fp_in), _O_BINARY);


    // carrier demodulator
    const float Fsymbol = 87e3;
    const float Fsample = 1e6;
    const float Faudio = Fsymbol/5.0f;

    const int block_size = 4096;
    const int audio_buffer_size = (int)std::ceil(Faudio);

    auto x_buffer = new std::complex<uint8_t>[block_size];
    auto x_in_buffer = new std::complex<float>[block_size];
    auto y_buffer = new std::complex<float>[block_size];

    CarrierDemodulatorSpecification spec;
    {
        const float PI = 3.1415f;

        spec.f_sample = Fsample; 
        spec.f_symbol = Fsymbol;
        spec.baseband_filter.cutoff = Fsymbol;
        spec.baseband_filter.M = 10;
        spec.ac_filter.k = 0.99999f;
        spec.agc.beta = 0.1f;
        spec.agc.initial_gain = 0.1f;
        spec.carrier_pll.f_center = 0e3;
        spec.carrier_pll.f_gain = 2.5e3;
        spec.carrier_pll.phase_error_gain = 8.0f/PI;
        spec.carrier_pll_filter.butterworth_cutoff = 5e3;
        spec.carrier_pll_filter.integrator_gain = 1000.0f;
        spec.ted_pll.f_gain = 5e3;
        spec.ted_pll.f_offset = 0e3;
        spec.ted_pll.phase_error_gain = 1.0f;
        spec.ted_pll_filter.butterworth_cutoff = 10e3;
        spec.ted_pll_filter.integrator_gain = 250.0f;
    }

    auto constellation = new SquareConstellation(4);

    auto frame_decoder = new FrameDecoder(block_size);
    auto scrambler = new AdditiveScrambler(SCRAMBLER_CODE);
    auto crc8_calc = new CRC8_Calculator(CRC8_POLY);
    auto vitdec = new ViterbiDecoder<encoder_decoder_type>(25);

    frame_decoder->descrambler = scrambler;
    frame_decoder->crc8_calc= crc8_calc;
    frame_decoder->vitdec = vitdec;

    auto frame_sync = new FrameSynchroniser();
    auto preamble_detector = new PreambleDetector(PREAMBLE_CODE, 4);
    frame_sync->constellation = constellation;
    frame_sync->frame_decoder = frame_decoder;
    frame_sync->preamble_detector = preamble_detector;

    auto demod = new CarrierToSymbolDemodulator(spec, constellation);
    auto demod_buffer = new CarrierToSymbolDemodulatorBuffers(block_size);
    demod->buffers = demod_buffer;

    uint16_t* pcm_buffer = new uint16_t[audio_buffer_size];
    int pcm_buffer_index = 0;

    auto payload_handler = 
        [&pcm_buffer, &pcm_buffer_index, audio_buffer_size]
        (uint8_t* x, const uint16_t N) 
    {
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

            pcm_buffer[pcm_buffer_index] = v1;
            pcm_buffer_index++;

            if (pcm_buffer_index == audio_buffer_size) {
                pcm_buffer_index = 0;
                fwrite(pcm_buffer, sizeof(uint16_t), audio_buffer_size, stdout);
            }
        }
    };

    size_t rd_block_size = 0;
    int rd_total_blocks = 0;
    while ((rd_block_size = fread(reinterpret_cast<uint8_t*>(x_buffer), 2*sizeof(uint8_t), block_size, fp_in)) > 0) {
        if (rd_block_size != block_size) {
            LOG_MESSAGE("Got mismatched block size after %d blocks\n", rd_total_blocks);
            break;
        }
        rd_total_blocks++; 

        for (int i = 0; i < block_size; i++) {
            const uint8_t I = x_buffer[i].real();
            const uint8_t Q = x_buffer[i].imag();
            x_in_buffer[i].real((float)I - 127.5f);
            x_in_buffer[i].imag((float)Q - 127.5f);
        }

        auto total_symbols = demod->ProcessBlock(x_in_buffer, y_buffer);
        for (int i = 0; i < total_symbols; i++) {
            const auto IQ = y_buffer[i];
            const auto res = frame_sync->process(IQ);
            auto payload = frame_decoder->GetPayload();

            using Res = FrameSynchroniser::Result;
            switch (res) {
            case Res::PREAMBLE_FOUND:
                LOG_MESSAGE(
                    "Preamble: phase %d, conflict=%d, desync=%d\n", 
                    preamble_detector->GetPhaseIndex(), 
                    preamble_detector->IsPhaseConflict(), 
                    preamble_detector->GetDesyncBitcount());
                break;
            case Res::BLOCK_SIZE_OK:
                LOG_MESSAGE("Got block size: %d\n", payload.length);
                break;
            case Res::BLOCK_SIZE_ERR:
                LOG_MESSAGE("Got invalid block size: %d\n", payload.length);
                break;
            case Res::PAYLOAD_ERR:
                LOG_MESSAGE("Got mismatching crc on payload\n");
                break;
            case Res::PAYLOAD_OK:
                payload_handler(payload.buf, payload.length);
                break;
            case Res::NONE:
            default:
                break;
            }
        }

    }

    fclose(fp_in);

    delete x_buffer;
    delete x_in_buffer;
    delete y_buffer;
    LOG_MESSAGE("Exiting\n");

    return 0;
}
