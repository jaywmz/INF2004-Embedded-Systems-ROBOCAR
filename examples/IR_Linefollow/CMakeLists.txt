# Specify the source file
add_executable(IR_Linefollow IR_Linefollow.c)

# Pull in the Pico SDK (pico_stdlib) for standard functions
target_link_libraries(IR_Linefollow pico_stdlib hardware_pwm hardware_adc hardware_timer)

# Enable USB output and UART output (for debugging or serial communication)
pico_enable_stdio_usb(IR_Linefollow 1)
pico_enable_stdio_uart(IR_Linefollow 0)

# Create map/bin/hex files, etc.
pico_add_extra_outputs(IR_Linefollow)