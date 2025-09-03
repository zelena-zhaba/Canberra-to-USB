# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Canberra-to-USB C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(Canberra-to-USB 
    Canberra-to-USB.c
    )

# Generate the PIO header file
pico_generate_pio_header(Canberra-to-USB ${CMAKE_CURRENT_LIST_DIR}/input.pio)

pico_set_program_name(Canberra-to-USB "Canberra-to-USB")
pico_set_program_version(Canberra-to-USB "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Canberra-to-USB 0)
pico_enable_stdio_usb(Canberra-to-USB 1)

# Add the standard library to the build
target_link_libraries(Canberra-to-USB
        pico_stdlib
        hardware_pio
        hardware_dma)

# Add the standard include files to the build
target_include_directories(Canberra-to-USB PRIVATE ${CMAKE_BINARY_DIR})

pico_add_extra_outputs(Canberra-to-USB)

