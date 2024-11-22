add_library(linefollow STATIC
    linefollow.c
    linefollow.h
)

# Pull in the Pico SDK (pico_stdlib) for standard functions
target_link_libraries(linefollow pico_stdlib hardware_pwm hardware_adc hardware_timer)

# Enable USB output and UART output (for debugging or serial communication)
pico_enable_stdio_usb(linefollow 1)
pico_enable_stdio_uart(linefollow 0)

# Create map/bin/hex files, etc.
pico_add_extra_outputs(linefollow)