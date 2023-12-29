cmake_minimum_required(VERSION 3.10)
project(rtlsdr_qam_demod)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(imgui REQUIRED)
find_package(implot REQUIRED)

if(MSVC)
    add_compile_options(/fp:fast /arch:AVX2 /MP)
else()
    add_compile_options(-march=native -ffast-math)
endif()

if(WIN32)
    # Ignore warnings about C runtime insecure functions (eg. fopen_s)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

# MSVC = vcpkg package manager
# MSYS2 + Ubuntu = package manager
if(WIN32)
    find_package(portaudio CONFIG REQUIRED)
    set(PORTAUDIO_LIBS portaudio)
    set(EXTRA_LIBS "")
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(portaudio REQUIRED IMPORTED_TARGET portaudio-2.0)
    set(PORTAUDIO_LIBS PkgConfig::portaudio)
    set(EXTRA_LIBS pthread)
endif()

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(DSP_DIR ${SRC_DIR}/dsp)
add_library(dsp_lib STATIC
    ${DSP_DIR}/filter_designer.cpp)
target_include_directories(dsp_lib PRIVATE ${DSP_DIR} ${SRC_DIR})
target_compile_features(dsp_lib PRIVATE cxx_std_17)

set(CONSTELLATION_DIR ${SRC_DIR}/constellation)
add_library(constellation_lib STATIC
    ${CONSTELLATION_DIR}/constellation.cpp)
target_include_directories(constellation_lib PRIVATE ${CONSTELLATION_DIR} ${SRC_DIR})
target_compile_features(constellation_lib PRIVATE cxx_std_17)

set(DEMOD_DIR ${SRC_DIR}/demodulator)
add_library(demod_lib STATIC
    ${DEMOD_DIR}/pll_mixer.cpp
    ${DEMOD_DIR}/qam_sync_buffers.cpp
    ${DEMOD_DIR}/qam_sync.cpp)
target_link_libraries(demod_lib PRIVATE dsp_lib constellation_lib)
target_include_directories(demod_lib PRIVATE ${DEMOD_DIR} ${SRC_DIR})
target_compile_features(demod_lib PRIVATE cxx_std_17)

set(DECODER_DIR ${SRC_DIR}/decoder)
add_library(decoder_lib STATIC
    ${DECODER_DIR}/convolutional_encoder.cpp
    ${DECODER_DIR}/frame_decoder.cpp
    ${DECODER_DIR}/phil_karn_viterbi_decoder.cpp
    ${DECODER_DIR}/viterbi_decoder.cpp
    ${DECODER_DIR}/preamble_detector.cpp)
target_link_libraries(decoder_lib PRIVATE constellation_lib)
target_include_directories(decoder_lib PRIVATE ${DECODER_DIR} ${SRC_DIR})
target_compile_features(decoder_lib PRIVATE cxx_std_17)

set(AUDIO_DIR ${SRC_DIR}/audio)
add_library(audio_lib STATIC
    ${AUDIO_DIR}/audio_mixer.cpp
    ${AUDIO_DIR}/portaudio_output.cpp
    ${AUDIO_DIR}/portaudio_utility.cpp
    ${AUDIO_DIR}/resampled_pcm_player.cpp)
target_link_libraries(audio_lib PRIVATE ${PORTAUDIO_LIBS})
target_include_directories(audio_lib PRIVATE ${DECODER_DIR} ${SRC_DIR})
target_compile_features(audio_lib PRIVATE cxx_std_17)

set(GETOPT_DIR ${SRC_DIR}/utility/getopt)
add_library(getopt STATIC ${GETOPT_DIR}/getopt.c)
target_include_directories(getopt PRIVATE getopt)

add_executable(read_data ${SRC_DIR}/read_data.cpp)
target_include_directories(read_data PRIVATE ${SRC_DIR})
target_link_libraries(read_data PRIVATE 
    demod_lib decoder_lib 
    audio_lib getopt ${PORTAUDIO_LIBS} ${EXTRA_LIBS})
target_compile_features(read_data PRIVATE cxx_std_17)

add_executable(view_data ${SRC_DIR}/view_data.cpp)
target_include_directories(view_data PRIVATE ${SRC_DIR})
target_link_libraries(view_data PRIVATE 
    imgui implot 
    demod_lib decoder_lib 
    audio_lib getopt ${PORTAUDIO_LIBS} ${EXTRA_LIBS})
target_compile_features(view_data PRIVATE cxx_std_17)

add_executable(simulate_transmitter ${SRC_DIR}/simulate_transmitter.cpp)
target_include_directories(simulate_transmitter PRIVATE ${SRC_DIR})
target_link_libraries(simulate_transmitter PRIVATE 
    demod_lib decoder_lib 
    getopt ${EXTRA_LIBS})
target_compile_features(simulate_transmitter PRIVATE cxx_std_17)

add_executable(replay_data ${SRC_DIR}/replay_data.cpp)
target_include_directories(replay_data PRIVATE ${SRC_DIR})
target_link_libraries(replay_data PRIVATE getopt)
target_compile_features(replay_data PRIVATE cxx_std_17)
