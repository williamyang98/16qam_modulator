/*
 * crc32.c
 *
 * Created: 13/08/2022 2:15:22 AM
 *  Author: acidi
 */ 

#include "crc32.h"

uint32_t CRC32_LUT[256] = {0};

uint32_t crc32_calculate(uint8_t* x, const int N) {
	uint32_t crc32 = 0;
	for (int i = 0; i < N; i++) {
		crc32 = crc32 ^ ((uint32_t)(x[i]) << 24);
		crc32 = (crc32 << 8) ^ CRC32_LUT[crc32 >> 24];
	}
	return crc32;
}

void crc32_generate_table(const uint32_t G) {
	for (uint32_t i = 0; i < 256; i++) {
		uint32_t crc32 = i << 24;
		for (int j = 0; j < 8; j++) {
			if ((crc32 & 0x80000000) != 0) {
				crc32 = crc32 << 1;
				crc32 = crc32 ^ G;
				} else {
				crc32 = crc32 << 1;
			}
		}
		CRC32_LUT[i] = crc32;
	}
}