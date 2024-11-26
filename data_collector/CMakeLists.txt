cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(data_collector C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(data_collector
        main.cpp
)

target_link_libraries(data_collector pico_stdlib)

pico_enable_stdio_usb(data_collector 1)
pico_enable_stdio_uart(data_collector 0)

pico_add_extra_outputs(data_collector)
