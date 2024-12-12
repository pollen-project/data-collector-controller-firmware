add_library(ina219 INTERFACE)

target_include_directories(ina219
    INTERFACE
    ./
)

target_sources(ina219
    INTERFACE
    driver_ina219.c
    driver_ina219_interface.c
    driver_ina219_basic.c
)

target_link_libraries(ina219
    INTERFACE
    pico_stdlib
    hardware_i2c
)
