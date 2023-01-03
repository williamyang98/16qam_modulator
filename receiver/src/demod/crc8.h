#pragma once

#include <stdint.h>

class CRC8_Calculator {
private:
    uint8_t lut[256] = {0};
    const uint8_t G; // generator polynomial without leading coefficient (msb left)
public:
    CRC8_Calculator(uint8_t _G): G(_G) {
        generate_table();
    }
    uint8_t process(uint8_t* x, const int N) {
        uint8_t crc8 = 0;

        for (int i = 0; i < N; i++) {
            crc8 = crc8 ^ x[i];
            uint8_t lut_idx = crc8;
            crc8 = lut[lut_idx];
        }
        return crc8;
    }
private:
    void generate_table(void) {
        for (int i = 0; i < 256; i++) {
            uint8_t crc8 = static_cast<uint8_t>(i);
            for (int j = 0; j < 8; j++) {
                if ((crc8 & 0x80) != 0) {
                    crc8 = crc8 << 1;
                    crc8 = crc8 ^ G;
                } else {
                    crc8 = crc8 << 1;
                }
            }
            lut[i] = crc8;
        }
    }
};