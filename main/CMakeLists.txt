# Car executable
add_executable(car car.c)

# Include directories for car
target_include_directories(car PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}            # Include the current directory (main)
    ${CMAKE_SOURCE_DIR}/ultrasonic       # Include ultrasonic directory for headers
    ${CMAKE_SOURCE_DIR}/wheel_encoder    # Include wheel_encoder directory for headers
    ${CMAKE_SOURCE_DIR}/pid_motor        # Include pid_motor directory for headers
)

# Link libraries for car executable
target_link_libraries(car
    pico_stdlib                          # Link Pico's standard libraries
    ultrasonic_lib                       # Link the ultrasonic library
    wheel_encoder_lib                    # Link the wheel_encoder library
    pid_motor_lib                        # Link the pid_motor library
    FreeRTOS-Kernel                      # Link FreeRTOS kernel
)

# Create map/bin/hex/uf2 files for the car program
pico_add_extra_outputs(car)

# Enable USB and UART output for stdio (for debugging purposes)
pico_enable_stdio_usb(car 1)
pico_enable_stdio_uart(car 1)
