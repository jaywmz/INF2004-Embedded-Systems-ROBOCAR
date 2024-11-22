add_executable(dashboard
        dashboard
)
target_compile_definitions(dashboard PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        NO
)
target_include_directories(dashboard PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(dashboard PRIVATE
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_mdns
        pico_stdlib
)
pico_enable_stdio_usb(dashboard 1)
pico_add_extra_outputs(dashboard)
