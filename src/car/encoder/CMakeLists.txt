# Setup project and the executable for hello_pwm example
add_library(encoder STATIC
    encoder.c
)

# Pull in common dependencies
target_link_libraries(encoder
    pico_stdlib
    hardware_pwm
    hardware_uart
)
