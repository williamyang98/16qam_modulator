# Introduction
[![windows-build](https://github.com/FiendChain/16qam_modulator/actions/workflows/16qam_receiver_windows.yml/badge.svg)](https://github.com/FiendChain/16qam_modulator/actions/workflows/16qam_receiver_windows.yml)
[![linux-build](https://github.com/FiendChain/16qam_modulator/actions/workflows/16qam_receiver_linux.yml/badge.svg)](https://github.com/FiendChain/16qam_modulator/actions/workflows/16qam_receiver_linux.yml)

Software radio receiver for 16QAM modulator.

## Applications
For a sampling rate ```F``` and symbol rate ```S```.

| Command | Description |
| --- | --- |
| ```rtl_sdr -f $F -s $S -b $BLOCK_SIZE -E direct2``` | Reads IQ samples from receiver using direct sampling |
| ```read_data -f $F -s $S``` | Demodulates samples from stdin or a file and plays back audio |
| ```view_data -f $F -s $S``` | Same as read_data except there is a GUI for adjusting settings and visualising data |
| ```simulate_transmitter -f $F -s $S``` | Generates IQ samples locally |
| ```replay_data -f $F``` | Replays IQ data in realtime |

## Usage scenarios
| Scenario | Command |
| --- | --- |
| Demodulate data from receiver | ```rtl_sdr -f $F -s $S -b $BLOCK_SIZE -E direct2 \| view_data -f $F -S $S``` |
| Demodulate data from simulator | ```simulate_transmitter -f $F -s $S \| view_data -f $F -S $S``` |

## Build
### Windows
1. Install Visual Studio C++ and setup development environment.
2. Install vcpkg.
3. ```./toolchains/windows/cmake_configure.sh```.
4. ```ninja -C build-windows```.

### Ubuntu
1. ```./toolchains/ubuntu/install_packages.sh```.
2. ```./toolchains/ubuntu/cmake_configure.sh```.
3. ```ninja -C build-ubuntu```.
