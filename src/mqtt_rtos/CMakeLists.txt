add_executable(pico_mqtt_rtos
        pico_mqtt_rtos.c
)
target_compile_definitions(pico_mqtt_rtos PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        NO_SYS=1 # don't want NO_SYS (generally this would be in your lwipopts.h)
)
target_include_directories(pico_mqtt_rtos PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../common # for our common FreeRTOSConfig
        ${PICO_LWIP_CONTRIB_PATH}/apps/ping
)
target_link_libraries(pico_mqtt_rtos
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_mdns
        pico_lwip_mqtt
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)
pico_enable_stdio_usb(pico_mqtt_rtos 1)
pico_add_extra_outputs(pico_mqtt_rtos)

# Copy the binary to the build directory
add_custom_command(TARGET pico_mqtt_rtos POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:pico_mqtt_rtos>
        ${CMAKE_BINARY_DIR}/pico-wifi.elf
)