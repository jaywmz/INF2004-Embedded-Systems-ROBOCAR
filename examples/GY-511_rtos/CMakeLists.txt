add_executable(gy_511_rtos
    gy_511_rtos.c
)
target_compile_definitions(gy_511_rtos PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    NO_SYS=1 # don't want NO_SYS (generally this would be in your lwipopts.h)
)
target_include_directories(gy_511_rtos PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../common # for our common FreeRTOSConfig
    ${PICO_LWIP_CONTRIB_PATH}/apps/ping
)
target_link_libraries(gy_511_rtos
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mdns
    pico_lwip_mqtt
    pico_binary_info
    hardware_i2c
    pico_stdlib
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)
pico_enable_stdio_usb(gy_511_rtos 1)
pico_add_extra_outputs(gy_511_rtos)
