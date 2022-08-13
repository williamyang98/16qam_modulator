## Instructions
We are using pipes to direct data between applications. 

Application | Description
| --- | --- |
get_live_data.sh    | Print raw IQ bytes from rtlsdr dongle
build/*/read_data   | Demodulates raw IQ bytes to 16bit PCM audio
build/*/view_data   | Same as read_data except it has a GUI to view telemetry
build/*/simulate_transmitter | Print raw IQ bytes containing modulated data
aplay_port.sh       | Uses VLC to play raw PCM data
get_test_sample.sh  | Save raw IQ bytes from rtlsdr dongle to PCM file 
fx.bat              | Helper script for building with MSVC on Windows 

## Usage
#### 1. To play audio without GUI

<code>./get_live_data.sh | build/Release/read_data.exe | ./aplay_port.sh</code>

#### 2. To play audio with telemetry GUI

<code>./get_live_data.sh | build/Release/view_data.exe | ./aplay_port.sh</code>

#### 3. To build the project

<code>fx build release build/*project_name*.vcprojx</code>