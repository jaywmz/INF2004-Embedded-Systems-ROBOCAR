add_executable(pid_motor_rtos
    pid_motor_rtos.c
)
target_compile_definitions(pid_motor_rtos PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    NO_SYS=1 # don't want NO_SYS (generally this would be in your lwipopts.h)
)
target_include_directories(pid_motor_rtos PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../common # for our common FreeRTOSConfig
    ${PICO_LWIP_CONTRIB_PATH}/apps/ping
)
target_link_libraries(pid_motor_rtos
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_pwm
    hardware_uart
    pico_lwip_mdns
    pico_lwip_mqtt
    pico_stdlib
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)
pico_enable_stdio_usb(pid_motor_rtos 1)
pico_add_extra_outputs(pid_motor_rtos)