cmake_minimum_required(VERSION 3.10)
project(rtlsdr_qam_demod)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(imgui REQUIRED)
find_package(implot REQUIRED)

set(SRC_FILES
    src/filters.h
    src/carrier_dsp_blocks.cpp
    src/carrier_dsp.cpp
)

add_executable(read_data 
    ${SRC_FILES}
    src/read_data.cpp)
include_directories(read_data src/)
set_target_properties(read_data PROPERTIES CXX_STANDARD 17)

add_executable(view_data 
    ${SRC_FILES}
    src/view_data.cpp)
include_directories(view_data src/)
set_target_properties(view_data PROPERTIES CXX_STANDARD 17)
target_link_libraries(view_data PUBLIC imgui implot)

add_executable(simulate_transmitter
    src/simulate_transmitter.cpp
    src/getopt/getopt.c)
include_directories(simulate_transmitter src/ src/getopt/)
set_target_properties(simulate_transmitter PROPERTIES CXX_STANDARD 17)

add_executable(replay_data
    src/replay_data.cpp
    src/getopt/getopt.c)
include_directories(replay_data src/ src/getopt)
set_target_properties(replay_data PROPERTIES CXX_STANDARD 17)

if (WIN32)
target_compile_options(read_data PRIVATE "/MP")
target_compile_options(view_data PRIVATE "/MP")
target_compile_options(simulate_transmitter PRIVATE "/MP")
target_compile_options(replay_data PRIVATE "/MP")
endif (WIN32)