#pragma once
#include <stdint.h>
#include <stdlib.h>

template <typename T>
int push_big_endian_byte(uint8_t* x, T y) {
    auto y_addr = reinterpret_cast<uint8_t*>(&y);
    const int N = sizeof(y) / sizeof(uint8_t);
    for (int i = 0; i < N; i++) {
        x[i] = y_addr[N-1-i];
    }
    return N;
}