# wheel_encoder/CMakeLists.txt
add_library(wheel_encoder_lib 
    wheel_encoder.c
) 

# Add include directories
target_include_directories(wheel_encoder_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pid_motor  # This allows finding pid_motor.h
)
 
target_link_libraries(wheel_encoder_lib 
    pico_stdlib          
    hardware_gpio        
    hardware_timer       
    FreeRTOS-Kernel      
    FreeRTOS-Kernel-Heap4  
    hardware_pwm
    pid_motor_lib        # Add this to link with pid_motor
) 

pico_enable_stdio_usb(wheel_encoder_lib 1) 
pico_enable_stdio_uart(wheel_encoder_lib 1) 