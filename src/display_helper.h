#ifndef DISPLAY_HELPER_H
#define DISPLAY_HELPER_H

#include "lidar_helper.h"

#include "pico/stdlib.h"
#include "utils.h"
#include "ws2812_helper.h"

#define NUM_DISP_ROWS 8
#define NUM_DISP_COLS 8

void displays_init();
void displays_project(int16_t results_mm[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS], bool reject_direction, int direction, bool filter);

// for the sake of compartmentalisation
void displays_get_collective_pixel_dists(int16_t collective_pixel_dists[NUM_SCREENS][NUM_DISP_ROWS][NUM_DISP_COLS]);
void displays_distance_to_color_write();

#endif
