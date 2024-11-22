add_subdirectory(motor)
add_subdirectory(udp)
add_subdirectory(encoder)
add_subdirectory(ultrasonic)
add_subdirectory(barcode)
add_subdirectory(linefollow)

# add_subdirectory(barcode)
# add_subdirectory(linefollow)
add_executable(car
    car.c
)

target_compile_definitions(car PRIVATE
    NO_SYS=1 # don't want NO_SYS (generally this would be in your lwipopts.h)
)

target_include_directories(car PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/udp
    ${CMAKE_CURRENT_LIST_DIR}/motor
    ${CMAKE_CURRENT_LIST_DIR}/encoder
    ${CMAKE_CURRENT_LIST_DIR}/ultrasonic
    ${CMAKE_CURRENT_LIST_DIR}/barcode
    ${CMAKE_CURRENT_LIST_DIR}/linefollow
)

target_link_libraries(car
    motor
    udp
    encoder
    ultrasonic
    barcode
    linefollow
    pico_stdlib
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

pico_enable_stdio_usb(car 1)
pico_add_extra_outputs(car)

# # Custom command to copy the executable to the bin directory
# add_custom_command(TARGET car POST_BUILD
# COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bin
# COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:car> ${CMAKE_BINARY_DIR}/bin
# COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/car.uf2 ${CMAKE_BINARY_DIR}/bin
# )

# # Custom target to ensure the custom command is executed
# add_custom_target(copy_car ALL DEPENDS car)