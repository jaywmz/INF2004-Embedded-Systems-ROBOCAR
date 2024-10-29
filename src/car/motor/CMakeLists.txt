# Setup project and the executable for hello_pwm example
add_library(motor STATIC
    motor.c
)

# Pull in common dependencies
target_link_libraries(motor pico_stdlib hardware_pwm hardware_uart)
