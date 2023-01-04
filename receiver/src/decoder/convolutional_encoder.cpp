#include "convolutional_encoder.h"

const uint16_t CONV_ENC_REG_MASK = 0b11'1111'1111;
const uint16_t CONV_ENC_POLY_MASK = 0b111;
const uint8_t XOR_COUNT_LUT[8] = {0,1,1,0, 1,0,0,1};

ConvolutionalEncoder::ConvolutionalEncoder(const uint8_t poly[CODE_RATE]) 
{
    for (int i = 0; i < CODE_RATE; i++) {
        G[i] = poly[i];
    }

    // Generate lookup table
	const uint8_t g1 = G[0] & CONV_ENC_POLY_MASK;
	const uint8_t g2 = G[1] & CONV_ENC_POLY_MASK;
	for (int i = 0; i < 1024; i++) {
		uint16_t y = 0;
		uint16_t x = (uint16_t)i;
		
		for (int i = 0; i < 8; i++) {
			y = y << 2;
			x = x << 1;
			
			const uint16_t reg = (x & 0b111'0000'0000) >> 8;
			y |= (uint16_t)(XOR_COUNT_LUT[(reg & g1) & CONV_ENC_POLY_MASK] << 1);
			y |= (uint16_t)(XOR_COUNT_LUT[(reg & g2) & CONV_ENC_POLY_MASK] << 0);
		}
		
		LOOKUP_TABLE[i] = y;
	}

}

uint16_t ConvolutionalEncoder::consume_byte(uint8_t x) {
    reg = (reg << 8) | (uint16_t)x;
    const uint16_t y = LOOKUP_TABLE[reg & CONV_ENC_REG_MASK];
    return y;
}