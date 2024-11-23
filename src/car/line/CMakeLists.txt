add_library(line STATIC
    line.c
    line.h
)

target_include_directories(line PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
)

# Pull in the Pico SDK (pico_stdlib) for standard functions
target_link_libraries(line
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_timer
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)