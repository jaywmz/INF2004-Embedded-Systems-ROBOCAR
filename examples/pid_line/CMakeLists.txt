# Define the executable (main source file)
add_executable(pid_line
    pid_line.c
)

# Link necessary libraries
target_link_libraries(pid_line pico_stdlib hardware_adc hardware_gpio)

# Enable USB or UART output, depending on your setup
pico_enable_stdio_usb(pid_line 1)  # Use USB for stdout
pico_enable_stdio_uart(pid_line 0) # Disable UART

# Create the UF2 file
pico_add_extra_outputs(pid_line)