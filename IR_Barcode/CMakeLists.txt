# Specify the source file
add_executable(IR_Barcode IR_Barcode.c)

# Pull in the Pico SDK (pico_stdlib) for standard functions
target_link_libraries(IR_Barcode pico_stdlib hardware_pwm hardware_adc hardware_timer)

# Enable USB output and UART output (for debugging or serial communication)
pico_enable_stdio_usb(IR_Barcode 1)
pico_enable_stdio_uart(IR_Barcode 1)

# Create map/bin/hex files, etc.
pico_add_extra_outputs(IR_Barcode)
