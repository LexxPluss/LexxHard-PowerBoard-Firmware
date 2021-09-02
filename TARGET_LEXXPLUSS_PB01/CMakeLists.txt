add_library(mbed-lexxpluss-pb01 INTERFACE)

target_sources(mbed-lexxpluss-pb01
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-lexxpluss-pb01
    INTERFACE
        .
)

target_link_libraries(mbed-lexxpluss-pb01 INTERFACE mbed-stm32f103xb)
