# Introduction
This is 16QAM transmitter code for the STM32F401CCU6.

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

# Development environment setup
Uses the following software/tools:
- CubeMX: For configuring the GPIO, peripherals and timers
- CubeIDE: For compiling and editing the code
- CubeProgrammer: For uploading the binary file to the STM32 via USART/ST-Link/USB

All of these were downloaded from the STM32 website, which requires a developer account. For archival purposes the installers can be downloaded from the following [MegaNZ link](https://mega.nz/folder/JVpiXQaL#3XEjQmK5Fs9hsVNtCRJdhA).

## Setting up CubeMX and CubeIDE
1. Run the installers for CubeMX and CubeIDE.
2. (Optional) Disable telemetry collection by reading install instructions.
3. Setup project for CubeIDE by opening application and going to ```File > Open projects from file system > Import source (Directory)```.
4. Set build type to Release by going to ```Project > Build configurations > Set active > Release```.
5. Build binary by going to ```Project > Build project```.
6. Binary should now be in ```Release``` or ```Debug``` folder as a ```.bin``` file.

## Setting up CubeProgrammer
1. Install the STM32CubeProgrammer from installer.
2. The USB drivers should be installed for the STLink device. If you have problems later on go to device manager and make the STLink device use the "STMicroelectronics STLink dongle Version: x.x.x" from manual driver list.
3. Open STM32CubeProgrammer and use the following configuration settings.

### Updating STLink firmware
1. Click on the ```Firmware upgrade``` button.
2. Wait for the ```STLinkUpgrade``` dialog window to open.
3. Reconnect the STLink while the dialog is open.
4. Click on ```Open in update mode``` until device information shows up in ```Current Firmware``` and ```ST-Link ID``` fields. (May require a few clicks).
5. Click ```Upgrade``` to upgrade the firmware.
6. Wait for firmware upgrade to be confirmed.
7. Close dialog window.
8. Reconnect the STLink device.

### Uploading binary to STM32Fxxx
The ST-Link configuration is as follows:

| Field | Value |
| --- | --- |
| Serial Number | ... |
| Port | SWD |
| Frequency (kHz) | 4000 |
| Mode | Normal |
| Access port | 0 |
| Reset mode | Software Reset |
| Speed | Reliable |
| Shared | **Enabled** |
| Debug in Low Power mode | True |

1. Set the above configuration for STLink uploader. (STLinkV2 was used).
2. Press connect button and wait for green "Connected" status.
3. Select the "Download" icon button from the left vertical navigation buttons.
4. Under ```Download > File path``` select the ```.bin``` file compiled by CubeIDE.
5. Check ```Verify programming``` and ```Run after programming``` checkboxes.
6. Click ```Start Programming``` button and wait for upload to finish and be confirmed.