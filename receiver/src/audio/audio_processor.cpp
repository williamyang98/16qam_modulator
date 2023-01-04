#include "audio_processor.h"
#include <stdio.h>
#include "dsp/filter_designer.h"

AudioProcessor::AudioProcessor(
    const int _output_length,
    const float _Fs)
: Fs(_Fs),
  output_buffer(_output_length),
  output_span(output_buffer),
  output_builder(output_span)
{
    {
        const int N_ac = TOTAL_TAPS_IIR_AC_COUPLE;
        ac_filter = std::make_unique<IIR_Filter<int16_t>>(N_ac);
        create_iir_ac_filter(ac_filter->get_b(), ac_filter->get_a(), 0.9999f);
    }

    {
        const float k = 50.0f/(Fs/2.0f);
        const float r = 0.9999f;
        const int N_notch = TOTAL_TAPS_IIR_SECOND_ORDER_NOTCH_FILTER;
        notch_filter = std::make_unique<IIR_Filter<int16_t>>(N_notch);
        create_iir_notch_filter(notch_filter->get_b(), notch_filter->get_a(), k, r);
    }
}

void AudioProcessor::ProcessFrame(const uint8_t* x, const int N) {
    tmp_buffer.resize(N);
    for (int i = 0; i < N; i++) {
        uint16_t v = x[i];
        v = v - 128;
        v = v * 64;
        tmp_buffer[i] = v;
    }

    ac_filter->process(tmp_buffer.data(), tmp_buffer.data(), N);
    // notch_filter->process(tmp_buffer.data(), tmp_buffer.data(), N);

    for (int i = 0; i < N; i++) {
        tmp_buffer[i] *= output_gain;
    }

    auto rd_buffer = tcb::span(tmp_buffer);
    while (!rd_buffer.empty()) {
        const auto nb_read = output_builder.ConsumeBuffer(tmp_buffer);
        rd_buffer = rd_buffer.subspan(nb_read);
        if (output_builder.IsFull()) {
            fwrite(output_buffer.data(), sizeof(int16_t), output_buffer.size(), stdout);
            output_builder.Reset();
        }
    }
}
