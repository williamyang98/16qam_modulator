#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include <chrono>
#include <thread>

#include "utility/getopt/getopt.h"

#if _WIN32
#include <io.h>
#include <fcntl.h>
#endif

void usage() {
    fprintf(stderr, 
        "replay_data, a application to replay pcm files in realtime\n"
        "\t[-f samplerate (default: 2MHz)]\n"
        "\t[-b block size (default: 65536)]\n"
        "\t[-i input filename (default: None)]\n"
        "\t    If no file is provided then stdin is used\n"
        "\t[-l (disable loop)]\n"
        "\t[-h (show help)]\n"
    );
}

int main(int argc, char** argv) {
    int opt; 
    float Fs = 2e6;
    int block_size = 65536;
    char* filename = NULL;
    bool is_loop = true;

    while ((opt = getopt_custom(argc, argv, "f:b:i:lh")) != -1) {
        switch (opt) {
        case 'f':
            Fs = (float)(atof(optarg));
            break;
        case 'b':
            block_size = (int)(atof(optarg));
            break;
        case 'i':
            filename = optarg;
            break;
        case 'l':
            is_loop = false;
            break;
        case 'h':
        default:
            usage();
            return 0;
        }
    }

    FILE* fp = stdin;
    if (filename != NULL) {
        fp = fopen(filename, "rb");
        if (fp == nullptr) {
            fprintf(stderr, "Failed to open file: %s\n", filename);
            return 1;
        }
    }

#if _WIN32
    // NOTE: Windows does extra translation stuff that messes up the file if this isn't done
    // https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/setmode?view=msvc-170
    _setmode(_fileno(stdout), _O_BINARY);
    _setmode(_fileno(fp), _O_BINARY);
#endif

    if (block_size <= 0) {
        fprintf(stderr, "Block size (%d) cannot be negative\n", block_size);
        return 1;
    }

    if (Fs < 0.0f) {
        fprintf(stderr, "Samplerate (%.3f) cannot be negative\n", Fs);
        return 1;
    }

    const float Ts =  1.0f/Fs;
    const float Tblock = Ts*(float)(block_size);
    const int T_block_microseconds = (int)ceil(1e6 * Tblock);

    const int nb_channels = 2;
    uint8_t* buf = (uint8_t*)malloc(sizeof(uint8_t)*block_size*nb_channels);
    const size_t elem_size = nb_channels*sizeof(uint8_t);

    auto dt_prev_tx = std::chrono::high_resolution_clock::now();

    size_t n;
    do {
        fseek(fp, 0, 0);
        while ((n = fread(buf, elem_size, block_size, fp)) > 0) {
            fwrite(buf, elem_size, n, stdout);

            auto dt_curr_tx = std::chrono::high_resolution_clock::now();
            const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(dt_curr_tx-dt_prev_tx);
            const int T_offset =  static_cast<int>(dt.count());
            const int delay = T_offset - T_block_microseconds;
            dt_prev_tx = dt_curr_tx;
            std::this_thread::sleep_for(std::chrono::microseconds((T_block_microseconds - delay)/2));
        }
    } while (is_loop);

    fclose(fp);
}