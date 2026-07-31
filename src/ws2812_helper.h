/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Basically just a repackaging of the ws2812 pi pico example */

#ifndef WS2812_HELPER_H
#define WS2812_HELPER_H

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"
#include "utils.h"

// used for initing the pio
#define IS_RGBW false

#define NUM_PIXELS 64
#define NUM_ROWS 8
#define NUM_COLS 8
#define NUM_SCREENS 6

#define WS2812_PIN_FRONT 4
#define WS2812_PIN_BACK 8
#define WS2812_PIN_LEFT 5
#define WS2812_PIN_RIGHT 9
#define WS2812_PIN_TOP 7
#define WS2812_PIN_BOTTOM 6

enum screen_enum {
    FRONT,
    BACK,
    LEFT,
    RIGHT,
    TOP,
    BOTTOM
};

void ws2812_init();
// Don't need a release, it's all about the LEDs

void ws2812_display_screens();
void ws2812_write_screen_pixel(uint screen, uint row, uint col, rgb_t rgb);
void ws2812_blank_screen(uint screen);

#endif