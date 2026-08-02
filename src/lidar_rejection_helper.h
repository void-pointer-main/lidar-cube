#ifndef LIDAR_REJECTION_HELPER_H
#define LIDAR_REJECTION_HELPER_H

#include "pico/stdlib.h"

void rejection_helper_init();

// find the display on which the cube is lying, so that we can ignore a set of lidars
void rejection_helper_update(float acc[3], bool *moving, int *rejected_direction);

#endif
