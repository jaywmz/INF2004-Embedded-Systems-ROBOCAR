add_executable(gy-511 gy-511.c)

target_link_libraries(gy-511 pico_stdlib pico_binary_info hardware_i2c)

pico_enable_stdio_usb(gy-511 1)

pico_add_extra_outputs(gy-511)