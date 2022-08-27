/*
 * crc8.h
 *
 *  Created on: 27 Aug. 2022
 *      Author: acidi
 */

#ifndef SRC_CRC8_H_
#define SRC_CRC8_H_

#include <stdint.h>

uint8_t crc8_calculate(uint8_t* x, const int N);
void crc8_generate_table(const uint8_t G);

#endif /* SRC_CRC8_H_ */
