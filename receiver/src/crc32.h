#pragma once

#include <stdint.h>

class CRC32_Calculator {
private:
    uint32_t lut[256] = {0};
    const uint32_t G; // generator polynomial without leading coefficient (msb left)
public:
    CRC32_Calculator(uint32_t _G): G(_G) {
        generate_table();
    }
    uint32_t process(uint8_t* x, const int N) {
        uint32_t crc32 = 0;

        const int shift = 24;

        for (int i = 0; i < N; i++) {
            crc32 = crc32 ^ ((uint32_t)(x[i]) << shift);
            uint8_t lut_idx = (crc32 >> shift) & 0xFF;
            crc32 = (crc32 << 8) ^ lut[lut_idx];
        }
        return crc32;
    }
private:
    void generate_table(void) {
        const uint32_t bitcheck = 1u<<31;
        const int shift = 24;        

        for (int i = 0; i < 256; i++) {
            uint32_t crc32 = static_cast<uint8_t>(i) << shift;
            for (int j = 0; j < 8; j++) {
                if ((crc32 & bitcheck) != 0) {
                    crc32 = crc32 << 1;
                    crc32 = crc32 ^ G;
                } else {
                    crc32 = crc32 << 1;
                }
            }
            lut[i] = crc32;
        }
    }
};