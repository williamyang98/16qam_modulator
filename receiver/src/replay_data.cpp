#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include <chrono>
#include <thread>

#include "getopt/getopt.h"


void usage() {
    fprintf(stderr, 
        "replay_data, a application to replay pcm files in realtime\n"
        "\t[-f samplerate (default: 1MHz)]\n"
        "\t[-b block size (default: 1024)]\n"
        "\t[-i input filename (default: None)]\n"
        "\t    If no file is provided then stdin is used\n"
        "\t[-l (toggle loop)]\n"
        "\t[-h (show help)]\n"
    );
}

int main(int argc, char** argv) {
    int opt; 
    float Fs = 1e6;
    int block_size = 1024;
    char* filename = NULL;
    bool is_loop = false;

    while ((opt = getopt(argc, argv, "f:b:i:lh")) != -1) {
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
            is_loop = true;
            break;
        case 'h':
        default:
            usage();
            return 0;
        }
    }

    FILE* fp = stdin;
    if (filename != NULL) {
        errno_t err = fopen_s(&fp, filename, "r");
        if (err != 0) {
            fprintf(stderr, "Failed to open file: %s\n", filename);
            return 1;
        }
    }

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