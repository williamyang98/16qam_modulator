## Introduction
This is 16QAM transmitter code for the STM32F401CCU6.

Uses the following software/tools:
- CubeMX: For configuring the GPIO, peripherals and timers
- CubeIDE: For compiling and editing the code
- CubeProgrammer: For uploading the binary file to the STM32 via USART/ST-Link/USB

## Pin layout
| Pin | Description |
| --- | --- |
| PA1 | Carrier inphase |
| PA5 | Carrier quadrature |
| PB7 | Data I1 |
| PB6 | Data I0 |
| PB5 | Data Q1 |
| PB4 | Data Q0 |
| PB1 | Analog In |
| PB8 | Encoding debug signal |
| PB9 | Transmit ISR debug signal |
| PC13 | ADC DMA buffer debug signal |
| PA8 | Symbol timer compare out |
| PA6 | ADC timer compare out |
| PA2 | USART2 TX |

## Specifications
| Parameter | Value |
| --- | --- |
| Carrier frequency | 4.2MHz |
| Symbol frequency | 200kHz |
| ADC frequency | 40kHz |

## Notes
For the USB debugging bootloader to work for uploading the code,
pulldown resistors needs to be added to PA9 and PA10. These correspond to 
the UART debugging interface used by the system bootloader for debugging and uploading.

For some reason the STM32F401CCU6 development board (black pill) doesn't include these pulldown resistors.

## Configuration
### Peripheral setup
- Timer 2 has channel 1 and 2 setup to output 4.2MHz quadrature for the local oscillator.
- Timer 1 is configured to run at 200kHz.
- Timer 3 is connected in slave mode to Timer 1 with a prescaler of 5, so it runs at 40kHz. 
- The ADC is set for regular conversions triggered by the TIM3_TRGO update event, so it too runs at 40kHz.
- The ADC has the DMA controller setup so that every reading is stored in our allocated buffer. 
  The DMA controller is setup to roll around so we end up with a circular buffer. 

### GPIO
- The ADC is reading off channel 9 (PB1)
- 4bit symbol output is on PB4 to PB7
- Timer 2 channel 1 and 2 is outputing on PA1, PA5
- Timer 1 channel 1 is outputing to PA8 for debugging.
- Timer 3 channel 1 is outputing to PA6 for debugging.
- The ADC DMA interrupt half and full buffer callbacks toggle PC13 (LED)
- The symbol transmit ISR pulses PB9 for debugging.
- The main loop encoding calculations pulses PB8 for debugging.
- PA2 is used for USART2 TX (Unused)

### Interrupt setup
- The ADC DMA interrupt is called when the buffer is half filled and fully filled. This is done so we can double buffer for simultaneous encoding and transmit.
- Timer 3 is used to clock the ADC regular conversion meaning it directly triggers the ADC DMA interrupt.
- Timer 1 has it's update event interrupt enabled so that it's ISR can be used to transmit the 4bit 16-QAM symbols.
- Timer 1 is given a higher priority (lower value) than the ADC DMA interrupt so the symbol frequency is constant without interruptions by ADC sampling.