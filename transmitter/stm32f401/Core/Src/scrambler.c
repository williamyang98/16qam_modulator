/*
 * scrambler.c
 *
 *  Created on: 27 Aug. 2022
 *      Author: acidi
 */

#include "scrambler.h"

uint16_t SCRAMBLER_SYNCWORD = 0;
uint16_t SCRAMBLER_REG = 0;

void scrambler_reset() {
	SCRAMBLER_REG = SCRAMBLER_SYNCWORD;
}


void scrambler_init(uint16_t syncword) {
	SCRAMBLER_SYNCWORD = syncword;
}

// NOTE: The scrambler is of type [14,15]
uint8_t scrambler_process(uint8_t x) {
	uint8_t mask = (SCRAMBLER_REG ^ (SCRAMBLER_REG << 1)) >> 8;
	SCRAMBLER_REG = (SCRAMBLER_REG << 8) | mask;
	return x ^ mask;
}
