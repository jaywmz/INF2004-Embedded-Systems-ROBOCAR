# Specify the source file
add_executable(pwm_adc pwm_adc.c)

# Pull in the Pico SDK (pico_stdlib) for standard functions
target_link_libraries(pwm_adc pico_stdlib hardware_pwm hardware_adc hardware_timer)

# Enable USB output and UART output (for debugging or serial communication)
pico_enable_stdio_usb(pwm_adc 1)
pico_enable_stdio_uart(pwm_adc 1)

# Create map/bin/hex files, etc.
pico_add_extra_outputs(pwm_adc)
