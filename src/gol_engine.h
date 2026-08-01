#ifndef GOL_ENGINE_H
#define GOL_ENGINE_H

#include "pico/stdlib.h"
#include "ws2812_helper.h"

void gol_init();
void gol_deinit();
void gol_update_and_write(int16_t dist_array[NUM_SCREENS][NUM_ROWS][NUM_COLS]);

#endif
