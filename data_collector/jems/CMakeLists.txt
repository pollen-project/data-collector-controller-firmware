add_library(jems INTERFACE)

target_include_directories(jems
    INTERFACE
    ./
)

target_sources(jems
    INTERFACE
    jems.c
)

target_link_libraries(jems
    INTERFACE
)
