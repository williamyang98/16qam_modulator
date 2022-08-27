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
| PA0 | Analog In |
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