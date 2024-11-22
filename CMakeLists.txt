cmake_minimum_required(VERSION 3.13)

# set(WIFI_SSID "zxcvf")
# set(WIFI_PASSWORD "12345678")

set(WIFI_SSID "LeoHotspot")
set(WIFI_PASSWORD "urwelcome@123")

set(COMPASS_MDNS "PicoW1D58.local") # Leo's Pico
set(CAR_MDNS)
set(DASHBOARD_MDNS "PicoW5725.local") # Elroy's Pico

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Do not allow in-source build.
if(${PROJECT_SOURCE_DIR} STREQUAL ${PROJECT_BINARY_DIR})
    message(FATAL_ERROR "In-source build is not allowed. Please build in a separate directory, such as ${PROJECT_SOURCE_DIR}/build.")
endif()

# Set global path variables.
# get_filename_component(__MODULE_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
# set(MODULE_ROOT_DIR ${__MODULE_ROOT_DIR} CACHE INTERNAL "coreMQTT repository root.")

# Set output directories.
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

# ====================================================================================
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(pico-wifi C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(examples)
add_subdirectory(src)