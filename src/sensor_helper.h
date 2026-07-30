#ifndef SENSOR_HELPER_H
#define SENSOR_HELPER_H

#include "pico/stdlib.h"
#include "vl53l7cx_api.h"
#include "PCF8575_helper.h"

#define VLX_NUM_ROWS 8
#define VLX_NUM_COLS 8

typedef struct {
    VL53L7CX_Configuration* cfg_ptr;
    uint16_t results_array[VLX_NUM_ROWS*VLX_NUM_COLS];
    volatile bool data_ready;
} lidar_t;

#endif
