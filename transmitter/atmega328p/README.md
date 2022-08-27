## Introduction
This is 16QAM transmitter code for the AtMega328p.
Uses Atmel/Microchip studio IDE.
Uses avrdude (from Arduino) for uploading the hex file.

## Pin layout
| Pin | Description |
| --- | --- |
| PB1/9 | Carrier inphase |
| PB2/10 | Carrier quadrature |
| PD7/7 | Data I1 |
| PD6/6 | Data I0 |
| PD5/5 | Data Q1 |
| PD4/4 | Data Q0 |
| PC0/A0 | Analog In |
| PB0/8 | Encoding debug signal |
| PB5/13 | Packet creation debug signal |
| PB3/11 | Transmit ISR debug signal |
| PB4/12 | ADC ISR debug signal |

## Specifications
| Parameter | Value |
| --- | --- |
| Carrier frequency | 4MHz |
| Symbol frequency | 87kHz |
| ADC frequency | 17.4kHz |
