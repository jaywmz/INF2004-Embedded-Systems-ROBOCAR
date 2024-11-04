
add_library(barcode STATIC
    barcode.c
    barcode.h
)

# Link necessary libraries
target_link_libraries(barcode pico_stdlib hardware_adc hardware_gpio)

# Enable USB or UART output, depending on your setup
pico_enable_stdio_usb(barcode 1)  # Use USB for stdout
pico_enable_stdio_uart(barcode 0) # Disable UART

# Create the UF2 file
pico_add_extra_outputs(barcode)