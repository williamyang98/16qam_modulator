#include "audio_processor.h"
#include <stdio.h>
#include "dsp/filter_designer.h"

AudioProcessor::AudioProcessor(
    const int _buffer_length, const int _frame_length,
    const float _Fs)
: buffer_length(_buffer_length), 
  frame_length(_frame_length),
  Fs(_Fs)
{
    input_buffer = new uint8_t[buffer_length];
    output_buffer = new int16_t[buffer_length];
    frame_buffer = new int16_t[frame_length];

    {
        const int N_ac = TOTAL_TAPS_IIR_AC_COUPLE;
        ac_filter = new IIR_Filter<int16_t>(N_ac);
        create_iir_ac_filter(ac_filter->get_b(), ac_filter->get_a(), 0.9999f);
    }

    {
        const float k = 50.0f/(Fs/2.0f);
        const float r = 0.9999f;
        const int N_notch = TOTAL_TAPS_IIR_SECOND_ORDER_NOTCH_FILTER;
        notch_filter = new IIR_Filter<int16_t>(N_notch);
        create_iir_notch_filter(notch_filter->get_b(), notch_filter->get_a(), k, r);
    }
}

AudioProcessor::~AudioProcessor() {
    delete [] input_buffer;
    delete [] output_buffer;
    delete [] frame_buffer;

    delete ac_filter;
    delete notch_filter;
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
    // notch_filter->process(frame_buffer, frame_buffer, N);

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
