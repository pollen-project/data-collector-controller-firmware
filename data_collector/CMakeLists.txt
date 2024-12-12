cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

project(data_collector C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(dht)
add_subdirectory(ina219)
add_subdirectory(jems)

add_executable(data_collector
        main.cpp
        MQTT.cpp
        MQTT.h
        gps.cpp
        gps.h
        Sensors.cpp
        Sensors.h
)

target_link_libraries(data_collector
        dht
        ina219
        jems
        pico_runtime
        pico_stdlib
        hardware_i2c
        hardware_pio
        hardware_rtc
        hardware_sleep
)

pico_enable_stdio_usb(data_collector 1)
pico_enable_stdio_uart(data_collector 0)

pico_add_extra_outputs(data_collector)
