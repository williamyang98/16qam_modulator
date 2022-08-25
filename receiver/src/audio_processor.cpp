#include "audio_processor.h"
#include <stdio.h>

AudioProcessor::AudioProcessor(const int _buffer_length, const int _frame_length)
: buffer_length(_buffer_length), frame_length(_frame_length) 
{
    input_buffer = new uint8_t[buffer_length];
    output_buffer = new int16_t[buffer_length];

    const float AC_FILTER_B[] = {1.0f, -1.0f};
    const float AC_FILTER_A[] = {1.0f, -0.999999f};
    ac_filter = new IIR_Filter<int16_t>(AC_FILTER_B, AC_FILTER_A, 2);
    frame_buffer = new int16_t[frame_length];
}

AudioProcessor::~AudioProcessor() {
    delete [] input_buffer;
    delete [] output_buffer;
    delete [] frame_buffer;
}

bool AudioProcessor::ProcessFrame(const uint8_t* x, const int N) {
    if (N > frame_length) {
        return false;
    }

    for (int i = 0; i < N; i++) {
        uint16_t v = x[i];
        v = v - 128;
        v = v * 64;
        frame_buffer[i] = v;
    }

    ac_filter->process(frame_buffer, frame_buffer, N);

    for (int i = 0; i < N; i++) {
        const uint8_t input_sample = x[i];
        const int16_t output_sample = frame_buffer[i] * output_gain;

        input_buffer[buffer_index] = input_sample;
        output_buffer[buffer_index] = output_sample;

        if (buffer_index == (buffer_length-1)) {
            fwrite(output_buffer, sizeof(int16_t), buffer_length, stdout);
        }
        buffer_index = (buffer_index+1) % buffer_length;
    }

    return true;
}