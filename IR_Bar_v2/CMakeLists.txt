# Add executable for your project
add_executable( IR_Bar_v2
    IR_Bar_v2.c
)
# Include FreeRTOS and necessary libraries
target_include_directories( IR_Bar_v2 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/lib/FreeRTOS/include            # FreeRTOS core headers
    ${PICO_SDK_PATH}/lib/FreeRTOS/portable/GCC/ARM_CM0   # FreeRTOS port headers
)
# Link necessary libraries
target_link_libraries( IR_Bar_v2
    pico_stdlib
    hardware_adc
    hardware_pwm 
    FreeRTOS-Kernel  # Use the correct name for the FreeRTOS kernel library
    FreeRTOS-Kernel-Heap4  # FreeRTOS kernel and dynamic heap
    c                      # Link standard C library explicitly
)
# Add USB or UART support depending on your stdio setup
pico_enable_stdio_usb( IR_Bar_v2 1)
pico_enable_stdio_uart(  IR_Bar_v2 0)
# Ensure the executable is flashed to the target device
pico_add_extra_outputs( IR_Bar_v2)