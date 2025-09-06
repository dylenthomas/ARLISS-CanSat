# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# VS Code Pico ext boilerplate
if(WIN32)
  set(USERHOME $ENV{USERPROFILE})
else()
  set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
  include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(fall_detection C CXX ASM)
pico_sdk_init()

# SD + FatFs repo (cloned alongside this file)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/fatfslib/FatFs_SPI)

add_executable(fall_detection
  fall_detection.c
  hw_config.c
)

add_library(gps ../libs/Neo6M_UBXParser/Neo6M_UBXParser.c)
target_include_directories(gps PUBLIC 
        ${CMAKE_CURRENT_LIST_DIR}/../libs/Neo6M_UBXParser
)

add_library(mpu ../libs/mpu6050/mpu6050.c)
target_include_directories(mpu PUBLIC 
        ${CMAKE_CURRENT_LIST_DIR}/../libs/mpu6050
)
target_link_libraries(mpu
        pico_stdlib
        hardware_i2c
)


pico_set_program_name(fall_detection "fall_detection")
pico_set_program_version(fall_detection "0.1")

# Use UART for logs (set USB to 1 instead if you prefer USB CDC)
pico_enable_stdio_uart(fall_detection 0)
pico_enable_stdio_usb(fall_detection 1)

target_include_directories(fall_detection PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/fatfslib/FatFs_SPI
  ${CMAKE_CURRENT_LIST_DIR}/fatfslib/FatFs_SPI/ff14a/source
)

target_link_libraries(fall_detection
  pico_stdlib
  hardware_spi
  FatFs_SPI
  gps
  mpu
)

pico_add_extra_outputs(fall_detection)
