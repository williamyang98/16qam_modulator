#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "getopt/getopt.h"

#include <Windows.h>
#include <io.h>
#include <fcntl.h>

#pragma comment(lib, "winmm.lib")

// Implementation of this pcm player is based off the following source:
// https://blog.csdn.net/weixinhum/article/details/29943973

void usage() {
    fprintf(stderr, 
        "pcm_play, plays 16bit pcm file\n\n"
        "Usage:\t[-h (show usage)]\n"
        "\t[-f sample rate (default: 17400Hz)]\n"
        "\t[-b block size (default: 8192)]\n"
    );
}

// https://docs.microsoft.com/en-us/previous-versions/dd743869(v=vs.85)
// When the buffer is completed, we set the length to 0
// This value is checked in the main thread where we busy wait with a double buffer
void CALLBACK wave_callback(
    HWAVEOUT hwo, UINT uMsg, 
    DWORD dwInstance, 
    LPWAVEHDR dwParam1, 
    DWORD dwParam2)
{  
    switch (uMsg)  {
    // https://docs.microsoft.com/en-us/windows/win32/multimedia/wom-done?redirectedfrom=MSDN
    case WOM_DONE:
        {
            LPWAVEHDR wave_header_ptr = dwParam1;
            wave_header_ptr->dwBufferLength = 0;
            break;  
        }
    }
}

int main(int argc, char** argv) {  
    FILE* fp = stdin;
    // NOTE: Windows does extra translation stuff that messes up the file if this isn't done
    // https://docs.microsoft.com/en-us/cpp/c-runtime-library/reference/setmode?view=msvc-170
    freopen(NULL, "rb", stdin);
    _setmode(fileno(fp), _O_BINARY);

    int Fsample = 87000/5;
    int block_size = 8192;
    
    int opt;
    while ((opt = getopt(argc, argv, "f:b:h")) != -1) {
        switch (opt) {
        case 'f':
            Fsample = static_cast<int>(atof(optarg));
            break;
        case 'b':
            block_size = static_cast<int>(atof(optarg));
            break;
        case 'h':
        case '?':
            usage();
            return 0;
        }
    }

    if (Fsample <= 0) {
        fprintf(stderr, "Sample rate must be a positive number (%d)\n", Fsample);
        return 1;
    }

    if (block_size <= 0) {
        fprintf(stderr, "Block size must be a positive number (%d)\n", block_size);
    }

    WAVEFORMATEX wave_format;
    wave_format.wFormatTag = WAVE_FORMAT_PCM;
    wave_format.nChannels = 1;
    wave_format.nSamplesPerSec = Fsample;
    wave_format.nAvgBytesPerSec = Fsample*2;
    wave_format.wBitsPerSample = 16;
    wave_format.nBlockAlign = 2;
    wave_format.cbSize = 0;

    HWAVEOUT wave_out;
    waveOutOpen(&wave_out, WAVE_MAPPER, &wave_format, (DWORD_PTR)wave_callback, 0L, CALLBACK_FUNCTION);

    WAVEHDR wave_header_1;
    wave_header_1.lpData = new char[block_size];  
    wave_header_1.dwBufferLength = block_size;
    wave_header_1.dwLoops = 0;
    wave_header_1.dwFlags = 0;

    WAVEHDR wave_header_2;
    wave_header_2.lpData = new char[block_size];  
    wave_header_2.dwBufferLength = block_size;
    wave_header_2.dwLoops = 0;
    wave_header_2.dwFlags = 0;

    WAVEHDR* active_buffer = &wave_header_1;
    WAVEHDR* inactive_buffer = &wave_header_2;

    // We are using dwBufferLength as a flag to communicate between
    // the main thread where we busy wait, and the callback which is
    // called whenever a buffer is done reading 
    active_buffer->dwBufferLength = 0;

    while (true) {
        while (true) {
            size_t nb_read = fread(inactive_buffer->lpData, 1, block_size, fp);
            if (nb_read == block_size) {
                break;
            }
            fprintf(stderr, "Expected %d samples got %d at buffer %p eof=%d\n", block_size, static_cast<int>(nb_read), inactive_buffer->lpData, feof(fp));
            Sleep(10);
            return 0;
        }

        // Write to the inactive buffer, and wait for the active buffer to be processed
        inactive_buffer->dwBufferLength = block_size;
        waveOutPrepareHeader(wave_out, inactive_buffer, sizeof(WAVEHDR));  
        waveOutWrite(wave_out, inactive_buffer, sizeof(WAVEHDR));  
    
        while (active_buffer->dwBufferLength > 0) {
            Sleep(1);
        }

        // Swap double buffers 
        WAVEHDR* tmp = active_buffer;
        active_buffer = inactive_buffer;
        inactive_buffer = tmp;
    }

    while ((active_buffer->dwBufferLength != 0) || (inactive_buffer->dwBufferLength != 0))
    {
        Sleep(50);
    }
  
    waveOutUnprepareHeader(wave_out, &wave_header_1, sizeof(WAVEHDR));
    waveOutUnprepareHeader(wave_out, &wave_header_2, sizeof(WAVEHDR));  

    delete [] wave_header_1.lpData;  
    delete [] wave_header_2.lpData;  

    fclose(fp);
    return 0;  
}
