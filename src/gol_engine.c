#include "gol_engine.h"

#include <string.h>
#include "utils.h"

// just to clean the indexing in cell_is_alive up
#define N (NUM_ROWS-1)

#define LOWER_ACTIVATION_THRESH_mm 10
#define UPPER_ACTIVATION_THRESH_mm 100

bool current_generation[NUM_SCREENS][NUM_ROWS][NUM_COLS] = {0};

static bool cell_will_be_alive(int screen, int row, int col);
static bool cell_is_alive(int screen, int row, int col);

void gol_init() {
    memset(current_generation, 0, sizeof(current_generation));
}

void gol_release() {

}

void gol_update_and_write(int16_t dist_array[NUM_SCREENS][NUM_ROWS][NUM_COLS]) {
    bool next_generation[NUM_SCREENS][NUM_ROWS][NUM_COLS] = {0};

    for (int s = 0; s < NUM_SCREENS; s++) {
        for (int r = 0; r < NUM_ROWS; r++) {
            for (int c = 0; c < NUM_COLS; c++) {
                next_generation[s][r][c] = cell_will_be_alive(s, r, c);
            }
        }
    }

    for (int s = 0; s < NUM_SCREENS; s++) {
        ws2812_blank_screen(s);
        for (int r = 0; r < NUM_ROWS; r++) {
            for (int c = 0; c < NUM_COLS; c++) {
                // normal progression
                current_generation[s][r][c] = next_generation[s][r][c];

                // additional forced user input
                if (dist_array[s][r][c] > LOWER_ACTIVATION_THRESH_mm && dist_array[s][r][c] < UPPER_ACTIVATION_THRESH_mm) {
                    current_generation[s][r][c] = true;
                }

                if (current_generation[s][r][c]) {
                    ws2812_write_screen_pixel(s, r, c, rgb2rgb_t_f(50, 50, 50));
                } else {
                    ws2812_write_screen_pixel(s, r, c, rgb2rgb_t_f(0, 20, 5));
                }
            }
        }
    }
}

static bool cell_will_be_alive(int screen, int row, int col) {
    int cell_count = 0;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0) {
                continue;
            }
            cell_count += cell_is_alive(screen, row+i, col+j);                        
        }
    }       
    
    if (cell_count < 2 || cell_count > 3)
        return false;

    if (cell_count == 3)
        return true;

    return cell_is_alive(screen, row, col);
}


static bool cell_is_alive(int screen, int row, int col) {
    // Degenerate case: diagonal corner outside the cube.
    if ((row == -1 && col == -1) ||
        (row == NUM_ROWS && col == NUM_COLS) ||
        (row == -1 && col == NUM_COLS) ||
        (row == NUM_ROWS && col == -1))
    {
        return false;
    }

    switch (screen) {
        case FRONT:
            if (row == -1)
                return current_generation[TOP][N][col];
            else if (row == NUM_ROWS)
                return current_generation[BOTTOM][0][col];
            else if (col == -1)
                return current_generation[LEFT][row][N];
            else if (col == NUM_COLS)
                return current_generation[RIGHT][row][0];
            else
                return current_generation[FRONT][row][col];

        case BACK:
            if (row == -1)
                return current_generation[TOP][0][N - col];
            else if (row == NUM_ROWS)
                return current_generation[BOTTOM][0][N - col];
            else if (col == -1)
                return current_generation[RIGHT][row][N];
            else if (col == NUM_COLS)
                return current_generation[LEFT][row][0];
            else
                return current_generation[BACK][row][col];

        case LEFT:
            if (row == -1)
                return current_generation[TOP][col][0];
            else if (row == NUM_ROWS)
                return current_generation[BOTTOM][N - col][0];
            else if (col == -1)
                return current_generation[BACK][row][N];
            else if (col == NUM_COLS)
                return current_generation[FRONT][row][0];
            else
                return current_generation[LEFT][row][col];

        case RIGHT:
            if (row == -1)
                return current_generation[TOP][N - col][N];
            else if (row == NUM_ROWS)
                return current_generation[BOTTOM][col][N];
            else if (col == -1)
                return current_generation[FRONT][row][N];
            else if (col == NUM_COLS)
                return current_generation[BACK][row][0];
            else
                return current_generation[RIGHT][row][col];

        case TOP:
            if (row == -1)
                return current_generation[BACK][0][N - col];
            else if (row == NUM_ROWS)
                return current_generation[FRONT][0][col];
            else if (col == -1)
                return current_generation[LEFT][0][row];
            else if (col == NUM_COLS)
                return current_generation[RIGHT][0][N - row];
            else
                return current_generation[TOP][row][col];

        case BOTTOM:
            if (row == -1)
                return current_generation[FRONT][N][col];
            else if (row == NUM_ROWS)
                return current_generation[BACK][N][N - col];
            else if (col == -1)
                return current_generation[LEFT][N][N - row];
            else if (col == NUM_COLS)
                return current_generation[RIGHT][N][row];
            else
                return current_generation[BOTTOM][row][col];

        default:
            return false;
    }
}