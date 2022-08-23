#pragma once

#include <stdint.h>

// https://en.wikipedia.org/wiki/Scrambler
// XOR's a source byte with an internal register
// The internal register is initialised with a syncword
// For each bit, the syncword is modified by XORing existing bits, and shifting them in
// The purpose of this is to scramble a source into seemingly white noise 
// so that symbols are not repeated and we can extract timinig information
class AdditiveScrambler {
private:
    uint16_t reg = 0;
    const uint16_t syncword = 0;
public:
    AdditiveScrambler(const uint16_t _syncword)
    : syncword(_syncword) 
    {
        reset();
    }

    void reset() {
        reg = syncword;
    }

    uint8_t process(uint8_t x) {
        uint16_t mask = reg ^ (reg << 1);
        mask = mask >> 8;
        reg = (reg << 8) | mask;

        return x ^ mask;
    }
};
