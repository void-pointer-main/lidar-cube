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
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Lidar_Cube C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# adding the VL53L7CX_ULD_API driver
add_subdirectory(${PROJECT_SOURCE_DIR}/VL53L7CX_ULD_driver_2.0.1)

# Add executable. Default name is the project name, version 0.1
add_executable(Lidar_Cube src/Lidar_Cube.c
        src/PCF8575_helper.c
        src/lidar_helper.c
        src/ws2812_helper.c
        src/display_helper.c
        src/hsluv.c
        src/utils.c
        src/ism_helper.c
        src/lidar_rejection_helper.c
        src/gol_engine.c
        src/mode_state_machine.c
        src/membrane_engine.c
        src/screen_saver.c
        ISM330IS-PID/ism330is_reg.c)

pico_set_program_name(Lidar_Cube "Lidar_Cube")
pico_set_program_version(Lidar_Cube "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Lidar_Cube 0)
pico_enable_stdio_usb(Lidar_Cube 1)

# generate the header file into the source tree as it is included in the RP2350 datasheet
file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

# Add the standard library to the build
target_link_libraries(Lidar_Cube
        pico_stdlib
        pico_multicore
        hardware_i2c
        hardware_irq
        hardware_pio
        hardware_dma
        vl53l7cx)

# Add the standard include files to the build
target_include_directories(Lidar_Cube PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src # for the header files
        ${CMAKE_CURRENT_LIST_DIR}/ISM330IS-PID
)

pico_add_extra_outputs(Lidar_Cube)

