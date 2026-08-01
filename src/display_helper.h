#ifndef DISPLAY_HELPER_H
#define DISPLAY_HELPER_H

#include "lidar_helper.h"

#include "pico/stdlib.h"
#include "utils.h"

#define NUM_DISP_ROWS 8
#define NUM_DISP_COLS 8

// Each display takes in 4 sensors
// From each sensor it only takes the half plane extending into the screen area
typedef enum {
    HP_TOP,
    HP_BOTTOM,
} half_plane_t;

enum lid_ref_edge {
    LREF_EDGE_TOP,
    LREF_EDGE_BOTTOM,
    LREF_EDGE_LEFT,
    LREF_EDGE_RIGHT,
    NUM_LREF_EDGES
};

// results array must be 8x8
typedef struct {
    uint results_array_index;
    half_plane_t half_plane;
} lidar_ref_t;

typedef struct {
    lidar_ref_t lid_refs[NUM_LREF_EDGES];
    int16_t pixel_dists[NUM_DISP_ROWS][NUM_DISP_COLS];
} display_t;

void displays_init();
void displays_project(int16_t results_mm[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS]);
void displays_distance_to_color_write();

#endif
