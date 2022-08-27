/*
 * scrambler.h
 *
 * Created: 13/08/2022 12:07:13 AM
 *  Author: acidi
 */ 


#ifndef SCRAMBLER_H_
#define SCRAMBLER_H_

#include <stdint.h>

void scrambler_reset();
void scrambler_init(uint16_t syncword);
uint8_t scrambler_process(uint8_t x);



#endif /* SCRAMBLER_H_ */