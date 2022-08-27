/*
 * conv_enc.h
 *
 *  Created on: 27 Aug. 2022
 *      Author: acidi
 */

#ifndef SRC_CONV_ENC_H_
#define SRC_CONV_ENC_H_

#include <stdint.h>


void conv_enc_calculate_table(const uint8_t G1, const uint8_t G2);
void conv_enc_reset(void);
int conv_enc_process(uint8_t* x, uint8_t* y,  const int N);

#endif /* SRC_CONV_ENC_H_ */
