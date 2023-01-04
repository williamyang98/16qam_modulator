#pragma once

#include <assert.h>
#include <stdint.h>

#define CODE_RATE 2

// R = 1/2
// K = 3
class ConvolutionalEncoder {
private:
    uint16_t reg = 0;
    uint8_t G[CODE_RATE];

    // 10bit lookup
    // 2bit = register state | 8bits = input
    // output of look up is the 16 bit encoded output
    uint16_t LOOKUP_TABLE[1024]; 
public:
    ConvolutionalEncoder(const uint8_t poly[CODE_RATE]);
    void reset(void) { reg = 0; }
    uint16_t consume_byte(uint8_t x); 
};