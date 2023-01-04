#pragma once

#include <stdint.h>
#include <memory>
#include <vector>
#include "dsp/iir_filter.h"
#include "utility/span.h"
#include "utility/reconstruction_buffer.h"

// Accepts an audio frame of 8bit data and appends it to an audio buffer
// When that audio buffer has reached the end, it will print to stdout the buffer as a chunk
// Then the index is reset to the beginning, and a new audio chunk is ready
class AudioProcessor {
public:
    int16_t output_gain = 3;
private:
    const float Fs;
    std::vector<int16_t> tmp_buffer;
    std::unique_ptr<IIR_Filter<int16_t>> ac_filter;
    std::unique_ptr<IIR_Filter<int16_t>> notch_filter;
    std::vector<int16_t> output_buffer;
    tcb::span<int16_t> output_span;
    ReconstructionBuffer<int16_t> output_builder;
public:
    AudioProcessor(const int _output_length, const float _Fs);
    void ProcessFrame(const uint8_t* x, const int N);
    int GetOutputBufferSize() { return (int)output_buffer.size(); }
    tcb::span<const int16_t> GetOutputBuffer() { return output_buffer; }
};