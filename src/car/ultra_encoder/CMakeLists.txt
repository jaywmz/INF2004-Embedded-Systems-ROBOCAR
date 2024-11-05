# Specify the executable for the hello_gpio_irq project
add_library(ultra_encoder_lib ultra_encoder.c
)

# Link the standard Pico SDK libraries, which include stdio and GPIO support
target_link_libraries(ultra_encoder_lib pico_stdlib hardware_pwm FreeRTOS-Kernel      
FreeRTOS-Kernel-Heap4)

# Enable USB output for stdio
pico_enable_stdio_usb(ultra_encoder_lib 1)

# (Optional) Enable UART output for stdio if needed, set to 1 if you want UART
pico_enable_stdio_uart(ultra_encoder_lib 0)

# Create the map, binary, and hex files for the executable
# pico_add_extra_outputs(wheel_ultra)

# Add URL for the example (you can change the URL to your repository or documentation if needed)
# example_auto_set_url(wheel_ultra)