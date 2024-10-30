# Setup project and the executable for hello_pwm example
add_library(pid_motor_lib
        pid_motor.c
        )

# Pull in common dependencies
target_link_libraries(pid_motor_lib pico_stdlib hardware_pwm hardware_uart)

# Enable USB and UART output
pico_enable_stdio_usb(pid_motor_lib 1)
pico_enable_stdio_uart(pid_motor_lib 1)

# Create map/bin/hex file etc.
# pico_add_extra_outputs(pid_motor)

# Add URL via pico_set_program_url (optional, can be removed)
# example_auto_set_url(pid_motor)
