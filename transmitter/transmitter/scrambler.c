/*
 * scrambler.c
 *
 * Created: 13/08/2022 12:07:03 AM
 *  Author: acidi
 */ 

#include "scrambler.h"

uint16_t SCRAMBLER_SYNCWORD = 0;
uint16_t SCRAMBLER_REG = 0;

uint8_t scrambler_shift_reg() {
	SCRAMBLER_REG = SCRAMBLER_REG << 1;
	uint8_t v = 0;
	v ^= ((SCRAMBLER_REG >> 14) & 0x1);
	v ^= ((SCRAMBLER_REG >> 15) & 0x1);
	SCRAMBLER_REG |= v;
	return v;
}

void scrambler_reset() {
	SCRAMBLER_REG = SCRAMBLER_SYNCWORD;
}


void scrambler_init(uint16_t syncword) {
	SCRAMBLER_SYNCWORD = syncword;
}

uint8_t scrambler_process(uint8_t x) {
	uint8_t y = 0;
	for (int i = 7; i >= 0; i--) {
		y |= scrambler_shift_reg() << i;
	}
	return x ^ y;
}