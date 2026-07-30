# we compile a static library
add_library(vl53l7cx STATIC
    VL53L7CX_ULD_API/src/vl53l7cx_api.c
    Platform/platform.c
)

target_include_directories(vl53l7cx PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/VL53L7CX_ULD_API/inc
    ${CMAKE_CURRENT_LIST_DIR}/Platform
)

target_link_libraries(vl53l7cx
    pico_stdlib
    hardware_i2c
)