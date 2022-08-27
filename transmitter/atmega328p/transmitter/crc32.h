/*
 * crc32.h
 *
 * Created: 13/08/2022 2:14:52 AM
 *  Author: acidi
 */ 


#ifndef CRC32_H_
#define CRC32_H_

#include <stdint.h>

uint32_t crc32_calculate(uint8_t* x, const int N);
void crc32_generate_table(const uint32_t G);

#endif /* CRC32_H_ */