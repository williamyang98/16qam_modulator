/*
 * conv_enc.c
 *
 *  Created on: 27 Aug. 2022
 *      Author: acidi
 */

#include "conv_enc.h"


const uint8_t CONV_ENC_REG_MASK  = 0b00111111;
const uint8_t CONV_ENC_POLY_MASK  = 0b111;

// our fast lookup table will have 6 bit lookup
// 2 bits = reg state 1:2 | 4 bits = input bits
// output of look up is the 8 bit encoded output
static uint8_t CONV_ENC_TABLE[64] = {0};
static uint8_t XOR_COUNT_LUT[8] = {0,1,1,0, 1,0,0,1};
static uint16_t CONV_ENC_REG = 0;

void conv_enc_calculate_table(const uint8_t G1, const uint8_t G2) {
	uint8_t g1 = G1 & CONV_ENC_POLY_MASK;
	uint8_t g2 = G2 & CONV_ENC_POLY_MASK;

	for (uint8_t i = 0; i < 0b01000000; i++) {
		uint8_t y = 0;
		uint8_t x = i;

		for (int i = 0; i < 4; i++) {
			y = y << 2;
			x = x << 1;

			uint8_t reg = (x & 0b01110000) >> 4;
			y |= XOR_COUNT_LUT[(reg & g1) & CONV_ENC_POLY_MASK] << 1;
			y |= XOR_COUNT_LUT[(reg & g2) & CONV_ENC_POLY_MASK] << 0;
		}

		CONV_ENC_TABLE[i] = y;
	}
}

void conv_enc_reset() {
	CONV_ENC_REG = 0;
}

int conv_enc_process(uint8_t* x, uint8_t* y, const int N) {
	int k = 0;
	for (int i = 0; i < N; i++) {
		CONV_ENC_REG |= x[i];
		y[k]   = CONV_ENC_TABLE[(CONV_ENC_REG >> 4) & CONV_ENC_REG_MASK];
		y[k+1] = CONV_ENC_TABLE[CONV_ENC_REG & CONV_ENC_REG_MASK];

		CONV_ENC_REG = CONV_ENC_REG << 8;

		k += 2;
	}

	return k;
}
