#include "membrane_engine.h"

#include <string.h>

#include "utils.h"

// just to clean the indexing in membrane_cell_height up
#define N (NUM_ROWS-1)

// we keep the next, current and previous iteration in the same array for convenience
#define NUM_KEPT_STEPS 3

// essentially defines the speed at which the waves will propagate
#define SIM_COEF 0.4f

#define DISSIPATION 0.99f
#define INPUT_COEF 0.05f

#define MAX_EXPECTED_MEMBRANE_DEFLECTION 300.f
#define BASE_INTENSITY 96u

float membrane_surface_height[NUM_KEPT_STEPS][NUM_SCREENS][NUM_ROWS][NUM_COLS];

static float membrane_cell_height(int screen, int row, int col, int step);
static float next_cell_height(int screen, int row, int col);

static rgb_t height_to_hot_cold_hue_rgb_t(float height);

void membrane_init() {
    memset(membrane_surface_height, 0, sizeof(membrane_surface_height));
}

void membrane_release() {

}

void membrane_update_and_write(int16_t dist_array[NUM_SCREENS][NUM_ROWS][NUM_COLS]) {
    static int16_t prev_dist_array[NUM_SCREENS][NUM_ROWS][NUM_COLS] = {0};

    // calculate the update
    for (int s = 0; s < NUM_SCREENS; s++) {
        for (int r = 0; r < NUM_ROWS; r++) {
            for (int c = 0; c < NUM_COLS; c++) {
                membrane_surface_height[0][s][r][c] = next_cell_height(s, r, c) * DISSIPATION + INPUT_COEF * (dist_array[s][r][c] - prev_dist_array[s][r][c]);

                ws2812_write_screen_pixel(s, r, c, height_to_hot_cold_hue_rgb_t(membrane_surface_height[0][s][r][c]));
            }
        }
    }

    // preparation for next update
    memcpy(prev_dist_array, dist_array, sizeof(prev_dist_array));
    for (int i = NUM_KEPT_STEPS-1; i > 0; i--) {
        memcpy(membrane_surface_height[i], membrane_surface_height[i-1], sizeof(membrane_surface_height[i]));
    }
}

static float next_cell_height(int screen, int row, int col) {
    return 2*membrane_cell_height(screen, row, col, 1) - membrane_cell_height(screen, row, col, 2)
    + SIM_COEF * SIM_COEF * (
        -4 * membrane_cell_height(screen, row, col, 1)
        + membrane_cell_height(screen, row+1, col, 1)
        + membrane_cell_height(screen, row-1, col, 1)
        + membrane_cell_height(screen, row, col+1, 1)
        + membrane_cell_height(screen, row, col-1, 1)
    );
}

static float membrane_cell_height(int screen, int row, int col, int step) {
    // Degenerate case: diagonal corner outside the cube.
    if ((row == -1 && col == -1) ||
        (row == NUM_ROWS && col == NUM_COLS) ||
        (row == -1 && col == NUM_COLS) ||
        (row == NUM_ROWS && col == -1))
    {
        return 0.f;
    }

    switch (screen) {
        case FRONT:
            if (row == -1)
                return membrane_surface_height[step][TOP][N][col];
            else if (row == NUM_ROWS)
                return membrane_surface_height[step][BOTTOM][0][col];
            else if (col == -1)
                return membrane_surface_height[step][LEFT][row][N];
            else if (col == NUM_COLS)
                return membrane_surface_height[step][RIGHT][row][0];
            else
                return membrane_surface_height[step][FRONT][row][col];

        case BACK:
            if (row == -1)
                return membrane_surface_height[step][TOP][0][N - col];
            else if (row == NUM_ROWS)
                return membrane_surface_height[step][BOTTOM][0][N - col];
            else if (col == -1)
                return membrane_surface_height[step][RIGHT][row][N];
            else if (col == NUM_COLS)
                return membrane_surface_height[step][LEFT][row][0];
            else
                return membrane_surface_height[step][BACK][row][col];

        case LEFT:
            if (row == -1)
                return membrane_surface_height[step][TOP][col][0];
            else if (row == NUM_ROWS)
                return membrane_surface_height[step][BOTTOM][N - col][0];
            else if (col == -1)
                return membrane_surface_height[step][BACK][row][N];
            else if (col == NUM_COLS)
                return membrane_surface_height[step][FRONT][row][0];
            else
                return membrane_surface_height[step][LEFT][row][col];

        case RIGHT:
            if (row == -1)
                return membrane_surface_height[step][TOP][N - col][N];
            else if (row == NUM_ROWS)
                return membrane_surface_height[step][BOTTOM][col][N];
            else if (col == -1)
                return membrane_surface_height[step][FRONT][row][N];
            else if (col == NUM_COLS)
                return membrane_surface_height[step][BACK][row][0];
            else
                return membrane_surface_height[step][RIGHT][row][col];

        case TOP:
            if (row == -1)
                return membrane_surface_height[step][BACK][0][N - col];
            else if (row == NUM_ROWS)
                return membrane_surface_height[step][FRONT][0][col];
            else if (col == -1)
                return membrane_surface_height[step][LEFT][0][row];
            else if (col == NUM_COLS)
                return membrane_surface_height[step][RIGHT][0][N - row];
            else
                return membrane_surface_height[step][TOP][row][col];

        case BOTTOM:
            if (row == -1)
                return membrane_surface_height[step][FRONT][N][col];
            else if (row == NUM_ROWS)
                return membrane_surface_height[step][BACK][N][N - col];
            else if (col == -1)
                return membrane_surface_height[step][LEFT][N][N - row];
            else if (col == NUM_COLS)
                return membrane_surface_height[step][RIGHT][N][row];
            else
                return membrane_surface_height[step][BOTTOM][row][col];

        default:
            return false;
    }
}

static rgb_t height_to_hot_cold_hue_rgb_t(float height) {
    rgb_t tmp;

    float x = height / MAX_EXPECTED_MEMBRANE_DEFLECTION;

    if (x > 1.f) {
        x = 1.f;
    } else if (x < -1.f) {
        x = -1.f;
    }

    if (x < 0.f) {
        float t = x + 1.0f;

        tmp.r = (uint8_t)(BASE_INTENSITY * t);
        tmp.g = (uint8_t)(BASE_INTENSITY * t);
        tmp.b = BASE_INTENSITY;
    } else {
        float t = 1.0f - x;

        tmp.r = BASE_INTENSITY;
        tmp.g = (uint8_t)(BASE_INTENSITY * t);
        tmp.b = (uint8_t)(BASE_INTENSITY * t);
    }

    return tmp;
}
