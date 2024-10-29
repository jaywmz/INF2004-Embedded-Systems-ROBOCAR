# Define the executable for the ultrasonic sensor program 
add_library(wheel_encoder_lib 
    wheel_encoder.c  # Add your ultrasonic source file here 
) 
 
# Pull in the Pico SDK's standard libraries (this includes stdio, GPIO, and more) 
target_link_libraries(wheel_encoder_lib 
    pico_stdlib          # Standard libraries 
    hardware_gpio        # GPIO control 
    hardware_timer       # Timer control 
    FreeRTOS-Kernel      # FreeRTOS kernel 
    FreeRTOS-Kernel-Heap4  # FreeRTOS heap 
    hardware_pwm           # PWM control
) 
 
# Create map/bin/hex/uf2 files for the program 
# pico_add_extra_outputs(wheel_encoder_lib) 
 
# Enable USB and UART output for stdio (so you can print debug messages to the serial console) 
pico_enable_stdio_usb(wheel_encoder_lib 1) 
pico_enable_stdio_uart(wheel_encoder_lib 1) 
 
# (Optional) Add URL for project example (can be removed if not needed) 
# example_auto_set_url(ultrasonic)