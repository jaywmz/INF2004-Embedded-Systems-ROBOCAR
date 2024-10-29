# Setup project and the executable for hello_pwm example
add_library(mqtt STATIC
    mqtt.c
)

target_compile_definitions(mqtt PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    NO_SYS=1 # don't want NO_SYS (generally this would be in your lwipopts.h)
)
target_include_directories(mqtt PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Pull in common dependencies
target_link_libraries(mqtt
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mdns
    pico_lwip_mqtt
    pico_stdlib
)
