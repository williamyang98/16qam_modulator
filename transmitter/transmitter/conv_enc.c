/*
 * conv_enc.c
 *
 * Created: 12/08/2022 11:45:57 PM
 *  Author: acidi
 */ 

#include "conv_enc.h"


const uint16_t CONV_ENC_REG_MASK  = 0b11100000000;
const uint8_t CONV_ENC_POLY_MASK  = 0b111;

uint8_t CONV_ENC_TABLE[8] = {0};
uint8_t XOR_COUNT_LUT[8] = {0,1,1,0, 1,0,0,1};
uint16_t CONV_ENC_REG = 0;

void conv_enc_calculate_table(const uint8_t G1, const uint8_t G2) {
	uint8_t g1 = G1 & CONV_ENC_POLY_MASK;
	uint8_t g2 = G2 & CONV_ENC_POLY_MASK;
	
	for (uint8_t i = 0; i < 8; i++) {
		uint8_t y = 0;
		y |= XOR_COUNT_LUT[(i & g1) & CONV_ENC_POLY_MASK] << 1;
		y |= XOR_COUNT_LUT[(i & g2) & CONV_ENC_POLY_MASK] << 0;
		
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
		
		//y[k] = x[i];
		//y[k+1] = x[i];
		
		y[k]   = 0;
		y[k+1] = 0;
		
		CONV_ENC_REG = CONV_ENC_REG << 1;
		y[k] |= CONV_ENC_TABLE[(CONV_ENC_REG & CONV_ENC_REG_MASK) >> 8] << 6;
		CONV_ENC_REG = CONV_ENC_REG << 1;
		y[k] |= CONV_ENC_TABLE[(CONV_ENC_REG & CONV_ENC_REG_MASK) >> 8] << 4;
		CONV_ENC_REG = CONV_ENC_REG << 1;
		y[k] |= CONV_ENC_TABLE[(CONV_ENC_REG & CONV_ENC_REG_MASK) >> 8] << 2;
		CONV_ENC_REG = CONV_ENC_REG << 1;
		y[k] |= CONV_ENC_TABLE[(CONV_ENC_REG & CONV_ENC_REG_MASK) >> 8];
		
		CONV_ENC_REG = CONV_ENC_REG << 1;
		y[k+1] |= CONV_ENC_TABLE[(CONV_ENC_REG & CONV_ENC_REG_MASK) >> 8] << 6;
		CONV_ENC_REG = CONV_ENC_REG << 1;
		y[k+1] |= CONV_ENC_TABLE[(CONV_ENC_REG & CONV_ENC_REG_MASK) >> 8] << 4;
		CONV_ENC_REG = CONV_ENC_REG << 1;
		y[k+1] |= CONV_ENC_TABLE[(CONV_ENC_REG & CONV_ENC_REG_MASK) >> 8] << 2;
		CONV_ENC_REG = CONV_ENC_REG << 1;
		y[k+1] |= CONV_ENC_TABLE[(CONV_ENC_REG & CONV_ENC_REG_MASK) >> 8];
		
		k += 2;
	}
	
	return k;
}