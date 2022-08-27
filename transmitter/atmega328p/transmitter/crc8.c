// CRC8 calculator
#include "crc8.h"

uint8_t CRC8_LUT[256] = {0};

uint8_t crc8_calculate(uint8_t* x, const int N) {
	uint8_t crc8 = 0;
	for (int i = 0; i < N; i++) {
		crc8 = crc8 ^ x[i];
		crc8 = CRC8_LUT[crc8];
	}
	return crc8;
}

void crc8_generate_table(const uint8_t G) {
	for (uint16_t i = 0; i < 256; i++) {
		uint8_t crc8 = i;
		for (int j = 0; j < 8; j++) {
			if ((crc8 & 0x80) != 0) {
				crc8 = crc8 << 1;
				crc8 = crc8 ^ G;
			} else {
				crc8 = crc8 << 1;
			}
		}
		CRC8_LUT[i] = crc8;
	}
}