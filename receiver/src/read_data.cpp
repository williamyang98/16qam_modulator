#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <assert.h>

#include "carrier_dsp.h"
#include "carrier_demodulator_spec.h"
#include "frame_synchroniser.h"
#include "qam_demodulator.h"
#include "audio_processor.h"

#include <io.h>
#include <fcntl.h>

#define PRINT_LOG 1

#if PRINT_LOG 
  #define LOG_MESSAGE(...) fprintf(stderr, ##__VA_ARGS__)
#else
  #define LOG_MESSAGE(...) (void)0
#endif

class FrameHandler: public QAM_Demodulator_Callback 
{
public:
    struct {
        int total = 0;
        int incorrect = 0;
        int correct = 0;
        int corrupted = 0;
        int repaired = 0;
        float GetPacketErrorRate() {
            return  (float)incorrect / (float)total;
        }
        float GetRepairFailureRate() {
            return (float)repaired / (float)correct;
        }
        void reset() {
            total = 0;
            incorrect = 0;
            correct = 0;
            corrupted = 0;
            repaired = 0;
        }
    } stats;
private:
    const int frame_length;
    AudioProcessor* audio;
public: 
    FrameHandler(AudioProcessor* _audio, const int _frame_length)
    : frame_length(_frame_length), audio(_audio) {}
public:
    virtual void OnFrameResult(
        FrameSynchroniser::Result res, 
        FrameDecoder::Payload payload)
    {
        using Res = FrameSynchroniser::Result;
        switch (res) {
        case Res::PREAMBLE_FOUND:
            // LOG_MESSAGE(
            //     "Preamble: phase %d, conflict=%d, desync=%d\n", 
            //     preamble_detector->GetPhaseIndex(), 
            //     preamble_detector->IsPhaseConflict(), 
            //     preamble_detector->GetDesyncBitcount());
            break;
        case Res::BLOCK_SIZE_OK:
            LOG_MESSAGE("Got block size: %d\n", payload.length);
            break;
        case Res::BLOCK_SIZE_ERR:
            LOG_MESSAGE("Got invalid block size: %d\n", payload.length);
            stats.corrupted++;
            break;
        case Res::PAYLOAD_ERR:
            LOG_MESSAGE("Got invalid block size: %d\n", payload.length);
            stats.total++;
            stats.incorrect++;
            break;
        case Res::PAYLOAD_OK:
            stats.total++;
            stats.correct++;
            if (payload.decoded_error > 0) {
                stats.repaired++;
            }
            if (payload.length == frame_length) {
                audio->ProcessFrame(payload.buf, payload.length);
            }
            break;
        case Res::NONE:
        default:
            break;
        }
    }
};

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
    _setmode(_fileno(fp_in), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);


    // carrier demodulator
    const float Fsymbol = 87e3;
    const float Fsample = 1e6;
    const float Faudio = Fsymbol/5.0f;

    const int block_size = 4096;
    const int audio_buffer_size = (int)std::ceil(Faudio);

    auto x_buffer = new std::complex<uint8_t>[block_size];

    const int audio_frame_length = 100;
    auto audio_processor = new AudioProcessor(audio_buffer_size, audio_frame_length);
    auto frame_handler = new FrameHandler(audio_processor, audio_frame_length);

    CarrierDemodulatorSpecification carrier_demod_spec;
    QAM_Demodulator_Specification qam_spec;
    {
        const float PI = 3.1415f;
        auto& spec = carrier_demod_spec;
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
    {
        auto& spec = qam_spec;
        spec.block_size = block_size;
        spec.scrambler_syncword = 0b1000010101011001;
        spec.preamble_code = 0b11111001101011111100110101101101;
        spec.crc8_polynomial = 0xD5;
        spec.downsample_filter.factor = 1;
        spec.downsample_filter.size = 0;
    }   


    auto constellation = new SquareConstellation(4);
    auto demod_buffer = new CarrierToSymbolDemodulatorBuffers(block_size);
    auto qam_demodulator = new QAM_Demodulator(carrier_demod_spec, qam_spec, constellation, demod_buffer);
    qam_demodulator->SetCallback(frame_handler);

    int rd_total_blocks = 0;
    while (true) {
        size_t rd_block_size = fread(x_buffer, sizeof(std::complex<uint8_t>), block_size, fp_in);
        if (rd_block_size != block_size) {
            LOG_MESSAGE("Got mismatched block size after %d blocks\n", rd_total_blocks);
            break;
        }
        rd_total_blocks++; 
        qam_demodulator->Process(x_buffer);
    }

    fclose(fp_in);

    LOG_MESSAGE("Exiting\n");
    delete x_buffer;
    delete demod_buffer;
    delete qam_demodulator;

    return 0;
}
