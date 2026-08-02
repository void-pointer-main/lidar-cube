#include "display_helper.h"

#include "string.h"

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
} display_t;

int16_t _collective_pixel_dists[NUM_SCREENS][NUM_DISP_ROWS][NUM_DISP_COLS] = {0};
display_t displays[NUM_SCREENS];

static void get_associated_indexes(int edge, int half_plane, int r, int c, int *vlx_r, int *vlx_c, int *d_r, int *d_c);

void displays_init() {
    ws2812_init(); // init hardware

    // we need to sow the displays together with their lidars
    displays[0].lid_refs[LREF_EDGE_TOP].results_array_index = 0;
    displays[0].lid_refs[LREF_EDGE_TOP].half_plane = HP_TOP;
    displays[0].lid_refs[LREF_EDGE_BOTTOM].results_array_index = 2;
    displays[0].lid_refs[LREF_EDGE_BOTTOM].half_plane = HP_TOP;
    displays[0].lid_refs[LREF_EDGE_LEFT].results_array_index = 3;
    displays[0].lid_refs[LREF_EDGE_LEFT].half_plane = HP_BOTTOM;
    displays[0].lid_refs[LREF_EDGE_RIGHT].results_array_index = 1;
    displays[0].lid_refs[LREF_EDGE_RIGHT].half_plane = HP_BOTTOM;

    displays[1].lid_refs[LREF_EDGE_TOP].results_array_index = 8;
    displays[1].lid_refs[LREF_EDGE_TOP].half_plane = HP_TOP;
    displays[1].lid_refs[LREF_EDGE_BOTTOM].results_array_index = 10;
    displays[1].lid_refs[LREF_EDGE_BOTTOM].half_plane = HP_TOP;
    displays[1].lid_refs[LREF_EDGE_LEFT].results_array_index = 9;
    displays[1].lid_refs[LREF_EDGE_LEFT].half_plane = HP_BOTTOM;
    displays[1].lid_refs[LREF_EDGE_RIGHT].results_array_index = 11;
    displays[1].lid_refs[LREF_EDGE_RIGHT].half_plane = HP_BOTTOM;

    displays[2].lid_refs[LREF_EDGE_TOP].results_array_index = 4;
    displays[2].lid_refs[LREF_EDGE_TOP].half_plane = HP_BOTTOM;
    displays[2].lid_refs[LREF_EDGE_BOTTOM].results_array_index = 7;
    displays[2].lid_refs[LREF_EDGE_BOTTOM].half_plane = HP_BOTTOM;
    displays[2].lid_refs[LREF_EDGE_LEFT].results_array_index = 11;
    displays[2].lid_refs[LREF_EDGE_LEFT].half_plane = HP_TOP;
    displays[2].lid_refs[LREF_EDGE_RIGHT].results_array_index = 3;
    displays[2].lid_refs[LREF_EDGE_RIGHT].half_plane = HP_TOP;

    displays[3].lid_refs[LREF_EDGE_TOP].results_array_index = 5;
    displays[3].lid_refs[LREF_EDGE_TOP].half_plane = HP_BOTTOM;
    displays[3].lid_refs[LREF_EDGE_BOTTOM].results_array_index = 6;
    displays[3].lid_refs[LREF_EDGE_BOTTOM].half_plane = HP_BOTTOM;
    displays[3].lid_refs[LREF_EDGE_LEFT].results_array_index = 1;
    displays[3].lid_refs[LREF_EDGE_LEFT].half_plane = HP_TOP;
    displays[3].lid_refs[LREF_EDGE_RIGHT].results_array_index = 9;
    displays[3].lid_refs[LREF_EDGE_RIGHT].half_plane = HP_TOP;

    displays[4].lid_refs[LREF_EDGE_TOP].results_array_index = 8;
    displays[4].lid_refs[LREF_EDGE_TOP].half_plane = HP_BOTTOM;
    displays[4].lid_refs[LREF_EDGE_BOTTOM].results_array_index = 0;
    displays[4].lid_refs[LREF_EDGE_BOTTOM].half_plane = HP_BOTTOM;
    displays[4].lid_refs[LREF_EDGE_LEFT].results_array_index = 4;
    displays[4].lid_refs[LREF_EDGE_LEFT].half_plane = HP_TOP;
    displays[4].lid_refs[LREF_EDGE_RIGHT].results_array_index = 5;
    displays[4].lid_refs[LREF_EDGE_RIGHT].half_plane = HP_TOP;

    displays[5].lid_refs[LREF_EDGE_TOP].results_array_index = 2;
    displays[5].lid_refs[LREF_EDGE_TOP].half_plane = HP_BOTTOM;
    displays[5].lid_refs[LREF_EDGE_BOTTOM].results_array_index = 10;
    displays[5].lid_refs[LREF_EDGE_BOTTOM].half_plane = HP_BOTTOM;
    displays[5].lid_refs[LREF_EDGE_LEFT].results_array_index = 7;
    displays[5].lid_refs[LREF_EDGE_LEFT].half_plane = HP_TOP;
    displays[5].lid_refs[LREF_EDGE_RIGHT].results_array_index = 6;
    displays[5].lid_refs[LREF_EDGE_RIGHT].half_plane = HP_TOP;
}

void displays_project(int16_t results_mm[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS]) {
    for (int i = 0; i < NUM_SCREENS; i++) {
        // Handle mixing of pixels
        for (int r = 0; r < NUM_DISP_ROWS; r++) {
            for (int c = 0; c < NUM_DISP_ROWS; c++) {
                _collective_pixel_dists[i][r][c] = MAX_DIST_MM;
            }
        }
        
        for (int edge = 0; edge < NUM_LREF_EDGES; edge++) {
            for (int r = 0; r < VLX_NUM_ROWS/2; r++) {
                for (int c = 0; c < VLX_NUM_COLS; c++) {
                    int vlx_r, vlx_c;
                    int d_r, d_c;
                    get_associated_indexes(edge, displays[i].lid_refs[edge].half_plane, r, c, &vlx_r, &vlx_c, &d_r, &d_c);

                    if (results_mm[displays[i].lid_refs[edge].results_array_index][vlx_r][vlx_c] < _collective_pixel_dists[i][d_r][d_c]) {
                        _collective_pixel_dists[i][d_r][d_c] = results_mm[displays[i].lid_refs[edge].results_array_index][vlx_r][vlx_c];
                    }
                }
            }
        }
    }
}

void displays_get_collective_pixel_dists(int16_t collective_pixel_dists[NUM_SCREENS][NUM_DISP_ROWS][NUM_DISP_COLS]) {
    memcpy(collective_pixel_dists, _collective_pixel_dists, sizeof(_collective_pixel_dists));
}

void displays_distance_to_color_write() {
    for (int i = 0; i < NUM_SCREENS; i++) {
        for (int r = 0; r < NUM_DISP_ROWS; r++) {
            for (int c = 0; c < NUM_DISP_ROWS; c++) {
                float t = (float)(_collective_pixel_dists[i][r][c]) / MAX_DIST_MM;
                // float t = (float)results_mm[i][r][c] / MAX_DIST_MM;
                // printf("%.2f\n", t);
                // t = powf(t, 0.6f); // possible adjusting of distance relation
                ws2812_write_screen_pixel(i, r, c, rgb_modified_intensity(distance_to_rgb_t_f(t), 6, 96));
            }
        }
    }
}

// we get the associated indexes for the the lidar and the display. This includes adding necessary offsets to both the lidar and display indexes.
static void get_associated_indexes(int edge, int half_plane, int r, int c, int *vlx_r, int *vlx_c, int *d_r, int *d_c) {

    *vlx_r = half_plane == HP_TOP ? r : 4+r;
    *vlx_c = c;

    switch (edge) {
        case LREF_EDGE_TOP:
            *d_r = half_plane == HP_TOP ? 3-r : r;
            *d_c = half_plane == HP_TOP ? 7-c : c;
            break;
        case LREF_EDGE_BOTTOM:
            *d_r = half_plane == HP_TOP ? 4+r : 7-r;
            *d_c = half_plane == HP_TOP ? c : 7-c;
            break;
        case LREF_EDGE_LEFT:
            *d_r = half_plane == HP_TOP ? c : 7-c;
            *d_c = half_plane == HP_TOP ? 3-r : r;
            break;
        case LREF_EDGE_RIGHT:
            *d_r = half_plane == HP_TOP ? 7-c : c;
            *d_c = half_plane == HP_TOP ? 4+r : 7-r;
            break;
        default:
            break;
    }
}
