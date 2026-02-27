# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

set(PICO_FLASH_SIZE_BYTES 16777216)
add_compile_definitions(PICO_FLASH_SIZE_BYTES=16777216)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_dashboard C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(./libraries/lvgl)
add_subdirectory(./libraries/bsp)
add_subdirectory(./libraries/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src)

include_directories(./libraries/lvgl)
include_directories(./libraries/bsp)

# Add executable. Default name is the project name, version 0.1

add_executable(pico_dashboard
        pico_dashboard.cpp
        lv_port/lv_port_disp.c
        lv_port/lv_port_indev.c
        lv_port/lv_port_fs.c
        ui/ui_dashboard.c
        ui/ui_debug_console.c
        protocol/invent_ems.c
)

pico_set_program_name(pico_dashboard "pico_dashboard")
pico_set_program_version(pico_dashboard "0.1")

# LVGL rendering + CAN ISR need more stack than the 2 KB default (SCRATCH_Y is 4 KB)
target_compile_definitions(pico_dashboard PRIVATE PICO_STACK_SIZE=0x1000)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_dashboard 0)
pico_enable_stdio_usb(pico_dashboard 0)

# Add the standard library to the build
target_link_libraries(pico_dashboard
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(pico_dashboard PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/lv_port
        ${CMAKE_CURRENT_LIST_DIR}/ui
        ${CMAKE_CURRENT_LIST_DIR}/protocol
)

# Add any user requested libraries
target_link_libraries(pico_dashboard
        hardware_spi
        hardware_i2c
        hardware_uart
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        pico_multicore
        lvgl
        bsp
        no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
        )

pico_add_extra_outputs(pico_dashboard)

