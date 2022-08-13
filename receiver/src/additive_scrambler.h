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

    template <typename T>
    T process(T x) {
        const int N = sizeof(T)*8;
        T mask = 0;
        for (int i = N-1; i >= 0; i--) {
            auto v = shift_reg();
            mask |= v << i;
        }
        return x ^ mask;
    }

    uint8_t shift_reg() {
        reg = reg << 1;
        uint8_t v = 0;
        v ^= (reg & (1 << 14)) >> 14;
        v ^= (reg & (1 << 15)) >> 15;
        reg |= v;
        return v;
    }
};
