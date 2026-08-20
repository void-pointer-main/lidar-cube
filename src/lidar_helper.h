#ifndef LIDAR_HELPER_H
#define LIDAR_HELPER_H

#include "pico/stdlib.h"

#define MAX_DIST_MM 1000u

#define NUM_LIDARS 12 // do not change unless you add manual assignements in the init function and INT interrupt handler

#define VLX_NUM_ROWS 8
#define VLX_NUM_COLS 8

int lidars_init();
void lidars_start_sampling();
void lidars_pause_sampling();
void lidars_sample(int16_t results[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS]);

#endif
