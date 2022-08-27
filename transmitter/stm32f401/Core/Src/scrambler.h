/*
 * scrambler.h
 *
 *  Created on: 27 Aug. 2022
 *      Author: acidi
 */

#ifndef SRC_SCRAMBLER_H_
#define SRC_SCRAMBLER_H_

#include <stdint.h>

void scrambler_reset();
void scrambler_init(uint16_t syncword);
uint8_t scrambler_process(uint8_t x);

#endif /* SRC_SCRAMBLER_H_ */
