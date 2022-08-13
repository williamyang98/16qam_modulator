/*
 * crc8.h
 *
 * Created: 12/08/2022 11:38:06 PM
 *  Author: acidi
 */ 


#ifndef CRC8_H_
#define CRC8_H_

#include <stdint.h>

uint8_t crc8_calculate(uint8_t* x, const int N);
void crc8_generate_table(const uint8_t G);


#endif /* CRC8_H_ */