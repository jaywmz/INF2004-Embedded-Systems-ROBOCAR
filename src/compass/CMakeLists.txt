add_executable(compass
    compass.c
)
target_compile_definitions(compass PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    DASHBOARD_MDNS=\"${DASHBOARD_MDNS}\"
    NO_SYS=1 # don't want NO_SYS (generally this would be in your lwipopts.h)
)
target_include_directories(compass PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(compass
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mdns
    pico_binary_info
    hardware_i2c
    pico_stdlib
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)
pico_enable_stdio_usb(compass 1)
pico_add_extra_outputs(compass)

# Custom command to copy the executable to the bin directory
add_custom_command(TARGET compass POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bin
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:compass> ${CMAKE_BINARY_DIR}/bin
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/compass.uf2 ${CMAKE_BINARY_DIR}/bin
)

# Custom target to ensure the custom command is executed
add_custom_target(copy_compass ALL DEPENDS compass)