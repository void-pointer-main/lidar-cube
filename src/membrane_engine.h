#ifndef MEMBRANE_ENGINE_H
#define MEMBRANE_ENGINE_H

#include "pico/stdlib.h"

#include "ws2812_helper.h"

/*
see https://beltoforion.de/en/recreational_mathematics/2d-wave-equation.php
*/

void membrane_init();
void membrane_release();
void membrane_update_and_write(int16_t dist_array[NUM_SCREENS][NUM_ROWS][NUM_COLS]);

#endif
