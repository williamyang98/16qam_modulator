#pragma once

#include <stdint.h>
#include <vector>
#include "utility/span.h"
#include "viterbi_config.h"

#define CODE_RATE 2

// Phil Karn's implementation
struct vitdec_t;

// A wrapper around the C styled api in Phil Karn's viterbi decoder implementation
class ViterbiDecoder
{
private:
    vitdec_t* vitdec;
    // We depuncture our encoded bits in blocks of max_depunctured_bits
    // COMPUTETYPE=int16_t
    std::vector<int16_t> depunctured_bits;
    const int max_decoded_bits;
    const int max_depunctured_bits;
public:
    // We are only handling a fixed code rate of 1/4
    // Refer to phil_karn_viterbi_decoder.cpp for the 1/4 decoder implementation
    // _input_bits = minimum number of bits in the resulting decoded message
    ViterbiDecoder(const uint8_t _poly[CODE_RATE], const int _input_bits);
    ~ViterbiDecoder();
    ViterbiDecoder(ViterbiDecoder&) = delete;
    ViterbiDecoder(ViterbiDecoder&&) = delete;
    ViterbiDecoder& operator=(ViterbiDecoder&) = delete;
    ViterbiDecoder& operator=(ViterbiDecoder&&) = delete;
    void Reset();
    void Update(tcb::span<const uint8_t> encoded_bytes);
    void GetTraceback(tcb::span<uint8_t> out_bytes);
    int16_t GetPathError(const int state=0);
};