add_library(IR_Bar_Line STATIC
    IR_Bar_Line.c
    IR_Bar_Line.h
)

target_include_directories(IR_Bar_Line PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
)

# Pull in the Pico SDK (pico_stdlib) for standard functions
target_link_libraries(IR_Bar_Line 
    pico_stdlib 
    hardware_pwm 
    hardware_adc 
    hardware_timer 
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap    
)