#pragma once

#include <stdint.h>
#include "filters.h"

// Accepts an audio frame of 8bit data and appends it to an audio buffer
// When that audio buffer has reached the end, it will print to stdout the buffer as a chunk
// Then the index is reset to the beginning, and a new audio chunk is ready
class AudioProcessor {
public:
    int16_t output_gain = 8;
private:
    const int buffer_length;
    const int frame_length;
    int buffer_index = 0;
    uint8_t* input_buffer = NULL;
    int16_t* output_buffer = NULL;
    int16_t* frame_buffer = NULL;
    IIR_Filter<int16_t>* ac_filter;
public:
    AudioProcessor(const int _buffer_length, const int _frame_length);
    ~AudioProcessor(); 
    bool ProcessFrame(const uint8_t* x, const int N);

    int GetFrameLength() const { return frame_length; }
    int GetBufferLength() const { return buffer_length; }
    auto GetInputBuffer() { return input_buffer; }
    auto GetOutputBuffer() { return output_buffer; }
};