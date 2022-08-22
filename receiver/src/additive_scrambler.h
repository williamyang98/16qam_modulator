#pragma once

#include <stdint.h>

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
