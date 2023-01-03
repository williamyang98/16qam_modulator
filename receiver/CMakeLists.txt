cmake_minimum_required(VERSION 3.10)
project(rtlsdr_qam_demod)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(imgui REQUIRED)
find_package(implot REQUIRED)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast /arch:AVX2")
    # NOTE: If we are on a platform without AVX2
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast")
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2 -mfma -ffast-math")
    # NOTE: If we are compiling with ssse3 not avx2+fma (affects pll and viterbi decoder)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3 -ffast-math")
endif()

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(DSP_DIR ${SRC_DIR}/dsp)
add_library(dsp_lib STATIC
    ${DSP_DIR}/filter_designer.cpp)
target_include_directories(dsp_lib PRIVATE ${DSP_DIR} ${SRC_DIR})
target_compile_features(dsp_lib PRIVATE cxx_std_17)

set(DEMOD_DIR ${SRC_DIR}/demod)
add_library(demod_lib STATIC
    ${DEMOD_DIR}/carrier_dsp_blocks.cpp
    ${DEMOD_DIR}/carrier_dsp.cpp
    ${DEMOD_DIR}/constellation.cpp
    ${DEMOD_DIR}/frame_decoder.cpp
    ${DEMOD_DIR}/preamble_detector.cpp
    ${DEMOD_DIR}/qam_demodulator.cpp
    ${DEMOD_DIR}/audio_processor.cpp)
target_link_libraries(demod_lib PRIVATE dsp_lib)
target_include_directories(demod_lib PRIVATE ${DEMOD_DIR} ${SRC_DIR})
target_compile_features(demod_lib PRIVATE cxx_std_17)

set(GETOPT_DIR ${SRC_DIR}/utility/getopt)
add_library(getopt STATIC ${GETOPT_DIR}/getopt.c)
target_include_directories(getopt PRIVATE getopt)

add_executable(read_data ${SRC_DIR}/read_data.cpp)
target_include_directories(read_data PRIVATE ${SRC_DIR})
target_link_libraries(read_data PRIVATE demod_lib getopt)
target_compile_features(read_data PRIVATE cxx_std_17)

add_executable(view_data ${SRC_DIR}/view_data.cpp)
target_include_directories(view_data PRIVATE ${SRC_DIR})
target_link_libraries(view_data PRIVATE imgui implot demod_lib getopt)
target_compile_features(view_data PRIVATE cxx_std_17)

add_executable(simulate_transmitter ${SRC_DIR}/simulate_transmitter.cpp)
target_include_directories(simulate_transmitter PRIVATE ${SRC_DIR})
target_link_libraries(simulate_transmitter PRIVATE demod_lib getopt)
target_compile_features(simulate_transmitter PRIVATE cxx_std_17)

add_executable(replay_data ${SRC_DIR}/replay_data.cpp)
target_include_directories(replay_data PRIVATE ${SRC_DIR})
target_link_libraries(replay_data PRIVATE getopt)
target_compile_features(replay_data PRIVATE cxx_std_17)

add_executable(pcm_play ${SRC_DIR}/pcm_play.cpp)
target_include_directories(pcm_play PRIVATE ${SRC_DIR})
target_link_libraries(pcm_play PRIVATE getopt)
target_compile_features(pcm_play PRIVATE cxx_std_17)

if (WIN32)
target_compile_options(dsp_lib PRIVATE "/MP")
target_compile_options(demod_lib PRIVATE "/MP")
target_compile_options(getopt PRIVATE "/MP")

target_compile_options(read_data PRIVATE "/MP")
target_compile_options(view_data PRIVATE "/MP")
target_compile_options(simulate_transmitter PRIVATE "/MP")
target_compile_options(replay_data PRIVATE "/MP")
target_compile_options(pcm_play PRIVATE "/MP")
endif (WIN32)
