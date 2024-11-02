# Define the executable for the ultrasonic sensor program
add_library(ultrasonic STATIC ultrasonic.c # Add your ultrasonic source file here
)

# Pull in the Pico SDK's standard libraries (this includes stdio, GPIO, and more)
target_link_libraries(ultrasonic
    pico_stdlib # Standard libraries
    hardware_gpio # GPIO control
    hardware_timer # Timer control
)