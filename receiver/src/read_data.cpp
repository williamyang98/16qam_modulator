#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

#if _WIN32
#include <io.h>
#include <fcntl.h>
#endif

#include "app.h"
#include "audio/portaudio_output.h"
#include "audio/resampled_pcm_player.h"
#include "audio/portaudio_utility.h"
#include "dsp/common.h"
#include "utility/getopt/getopt.h"

void usage() {
    fprintf(stderr, 
        "read_data, runs 16QAM demodulation on raw IQ values\n\n"
        "\t[-f sample rate (default: 1MHz)]\n"
        "\t[-s symbol rate (default: 200kHz)]\n"
        "\t[-b block size (default: 8192)]\n"
        "\t[-D downsample factor (default: 2)]\n"
        "\t[-S upsample factor (default: 4)]\n"
        "\t    rd_block_size = D*block_size\n"
        "\t    us_block_size = S*block_size\n"
        "\t    rd_block_size -> block_size -> us_block_size\n"
        "\t[-i input filename (default: None)]\n"
        "\t    If no file is provided then stdin is used\n"
        "\t[-g audio gain (default: 100)]\n"
        "\t[-A toggle audio output (default: true)]\n"
        "\t[-h (show usage)]\n"
    );
}

int main(int argc, char **argv) {
    int ds_factor = 2;
    int us_factor = 4;

    int demod_block_size = 8192;
    float Fsample = 1e6;
    float Fsymbol = 200e3;
    char* filename = NULL;

    int audio_gain = 100;
    // audio stream is symbol_rate / N
    const int audio_packet_sampling_ratio = 5;
    bool is_output_audio = true;

    int opt; 
    while ((opt = getopt_custom(argc, argv, "f:s:b:D:S:i:g:Ah")) != -1) {
        switch (opt) {
        case 'f':
            Fsample = (float)(atof(optarg));
            if (Fsample <= 0) {
                fprintf(stderr, "Sampling rate must be positive (%.2f)\n", Fsample); 
                return 1;
            }
            break;
        case 's':
            Fsymbol = (float)(atof(optarg));
            if (Fsymbol <= 0) {
                fprintf(stderr, "Symbol rate must be positive (%.2f)\n", Fsymbol); 
                return 1;
            }
            break;
        case 'b':
            demod_block_size = (int)(atof(optarg));
            if (demod_block_size <= 0) {
                fprintf(stderr, "Block size must be positive (%d)\n", demod_block_size); 
                return 1;
            }
            break;
        case 'D':
            ds_factor = (int)(atof(optarg));
            if (ds_factor <= 0) {
                fprintf(stderr, "Downsampling factor must be positive (%d)\n", ds_factor); 
                return 1;
            }
            break;
        case 'S':
            us_factor = (int)(atof(optarg));
            if (us_factor <= 0) {
                fprintf(stderr, "Upsampling factor must be positive (%d)\n", us_factor); 
                return 1;
            }
            break;
        case 'i':
            filename = optarg;
            break;
        case 'g':
            audio_gain = (int)(atof(optarg));
            if (audio_gain < 0) {
                fprintf(stderr, "Audio gain must be positive (%d)\n", audio_gain); 
                return 1;
            }
            break;
        case 'A':
            is_output_audio = false;
            break;
        case 'h':
        default:
            usage();
            return 0;
        }
    }

    audio_gain = dsp::clamp(audio_gain, 0, 1000);

    FILE* fp_in = stdin;
    if (filename != NULL) {
        fp_in = fopen(filename, "rb");
        if (fp_in == nullptr) {
            fprintf(stderr, "Failed to open file: %s\n", filename);
            return 1;
        }
    }

#if _WIN32
    // NOTE: Windows does extra translation stuff that messes up the file if this isn't done
    // https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/setmode?view=msvc-170
    _setmode(_fileno(fp_in), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif

    const float Faudio = Fsymbol/(float)audio_packet_sampling_ratio;
    const int audio_buffer_size = (int)Faudio;
    const int decoder_block_size = 1024;

    auto app = App(
        fp_in, demod_block_size, 
        decoder_block_size, ds_factor, us_factor, 
        audio_buffer_size, Faudio);

    {
        const float PI = 3.1415f;
        auto& spec = app.qam_sync_spec;
        spec.f_sample = Fsample; 
        spec.f_symbol = Fsymbol;
        
        spec.downsampling_filter.M = ds_factor;
        spec.downsampling_filter.K = 6;

        spec.upsampling_filter.L = us_factor;
        spec.upsampling_filter.K = 6;

        spec.ac_filter.k = 0.99999f;
        spec.agc.beta = 0.2f;
        spec.agc.initial_gain = 0.1f;
        spec.carrier_pll.f_center = 0e3;
        spec.carrier_pll.f_gain = 2.5e3;
        spec.carrier_pll.phase_error_gain = 8.0f/PI;
        spec.carrier_pll_filter.butterworth_cutoff = 5e3;
        spec.carrier_pll_filter.integrator_gain = 1000.0f;
        spec.ted_pll.f_gain = 30e3;
        spec.ted_pll.f_offset = 0e3;
        spec.ted_pll.phase_error_gain = 1.0f;
        spec.ted_pll_filter.butterworth_cutoff = 60e3;
        spec.ted_pll_filter.integrator_gain = 250.0f;
    }

    // Setup audio
    auto pa_handler = ScopedPaHandler();
    PaDeviceList pa_devices;
    PortAudio_Output pa_output;
    std::unique_ptr<Resampled_PCM_Player> pcm_player;
    {
        auto& mixer = pa_output.GetMixer();
        auto buf = mixer.CreateManagedBuffer(4);
        auto Fs = pa_output.GetSampleRate();
        pcm_player = std::make_unique<Resampled_PCM_Player>(buf, Fs);

        #ifdef _WIN32
        const auto target_host_api_index = Pa_HostApiTypeIdToHostApiIndex(PORTAUDIO_TARGET_HOST_API_ID);
        const auto target_device_index = Pa_GetHostApiInfo(target_host_api_index)->defaultOutputDevice;
        pa_output.Open(target_device_index);
        #else
        pa_output.Open(Pa_GetDefaultOutputDevice());
        #endif
    }

    pa_output.GetMixer().GetOutputGain() = (float)audio_gain / 100.0f;

    app.GetFrameHandler().is_output_audio = is_output_audio;

    app.GetAudioFilter().OnOutputBlock().Attach([&pcm_player, Faudio](tcb::span<const Frame<float>> data) {
        pcm_player->SetInputSampleRate((int)Faudio);
        pcm_player->ConsumeBuffer(data);
    });
    
    app.BuildDemodulator();
    app.Run();

    return 0;
}
