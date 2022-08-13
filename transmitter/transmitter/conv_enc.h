/*
 * conv_enc.h
 *
 * Created: 12/08/2022 11:45:47 PM
 *  Author: acidi
 */ 


#ifndef CONV_ENC_H_
#define CONV_ENC_H_

#include <stdint.h>


void conv_enc_calculate_table(const uint8_t G1, const uint8_t G2);
void conv_enc_reset(void);
int conv_enc_process(uint8_t* x, uint8_t* y,  const int N);



#endif /* CONV_ENC_H_ */