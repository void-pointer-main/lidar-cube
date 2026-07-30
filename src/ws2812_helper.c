/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Basically just a repackaging of the ws2812 pi pico example */

#include "ws2812_helper.h"
#include <math.h>
#include "hardware/dma.h"

typedef struct {
    PIO pio;
    uint sm;
    uint offset;
    uint pin;
    rgb_t _pixels[NUM_ROWS][NUM_COLS];
    bool row_col_flipped_before_inversion;
    bool row_inverted;
    bool col_inverted;
    uint dma_chan;
} screen_t;

screen_t screens[NUM_SCREENS];

#define PANEL_DISTANCE_FROM_CENTER 3.6f
#define PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL 2.8f
#define PIXEL2PIXEL_INCREMENT (2.8f*2/NUM_ROWS)

static inline uint32_t rgb_t2grb_u32(rgb_t pixel);

void ws2812_init() {
    uint offset_0 = pio_add_program(pio0, &ws2812_program);
    if (offset_0 == PICO_ERROR_GENERIC){
        while(1) printf("pio0 load fail\n");
    }

    uint offset_1 = pio_add_program(pio1, &ws2812_program);
    if (offset_1 == PICO_ERROR_GENERIC){
        while(1) printf("pio1 load fail\n");
    }

    // look, this part isn't very elegant, but... eh.

    // There are only four state machines per PIO block
    screens[TOP].pio = pio0;
    screens[BOTTOM].pio = pio0;
    screens[LEFT].pio = pio0;
    screens[RIGHT].pio = pio0;
    screens[FRONT].pio = pio1;
    screens[BACK].pio = pio1;

    screens[TOP].sm = 0;
    screens[BOTTOM].sm = 1;
    screens[LEFT].sm = 2;
    screens[RIGHT].sm = 3;
    screens[FRONT].sm = 0;
    screens[BACK].sm = 1;

    screens[TOP].offset = offset_0;
    screens[BOTTOM].offset = offset_0;
    screens[LEFT].offset = offset_0;
    screens[RIGHT].offset = offset_0;
    screens[FRONT].offset = offset_1;
    screens[BACK].offset = offset_1;

    screens[TOP].pin = WS2812_PIN_TOP;
    screens[BOTTOM].pin = WS2812_PIN_BOTTOM;
    screens[LEFT].pin = WS2812_PIN_LEFT;
    screens[RIGHT].pin = WS2812_PIN_RIGHT;
    screens[FRONT].pin = WS2812_PIN_FRONT;
    screens[BACK].pin = WS2812_PIN_BACK;

    screens[TOP].row_col_flipped_before_inversion = true;
    screens[BOTTOM].row_col_flipped_before_inversion = true;
    screens[LEFT].row_col_flipped_before_inversion = false;
    screens[RIGHT].row_col_flipped_before_inversion = false;
    screens[FRONT].row_col_flipped_before_inversion = false;
    screens[BACK].row_col_flipped_before_inversion = false;

    screens[TOP].row_inverted = false;
    screens[BOTTOM].row_inverted = false;
    screens[LEFT].row_inverted = true;
    screens[RIGHT].row_inverted = true;
    screens[FRONT].row_inverted = true;
    screens[BACK].row_inverted = true;

    screens[TOP].col_inverted = true;
    screens[BOTTOM].col_inverted = true;
    screens[LEFT].col_inverted = true;
    screens[RIGHT].col_inverted = true;
    screens[FRONT].col_inverted = true;
    screens[BACK].col_inverted = true;

    // for (int i = 0; i < NUM_SCREENS; i++) {
    //     screens[i].dma_chan = dma_claim
    // }

    for (int i = 0; i < NUM_SCREENS; i++) {
        ws2812_program_init(screens[i].pio, screens[i].sm, screens[i].offset, screens[i].pin, 800000, IS_RGBW);
    }
}

void ws2812_display_screens() {
    for (int hw_r = 0; hw_r < NUM_ROWS; hw_r++) {
        for (int hw_c = 0; hw_c < NUM_COLS; hw_c++) {
            for (int k = 0; k < NUM_SCREENS; k++) {
                pio_sm_put_blocking(screens[k].pio, screens[k].sm, rgb_t2grb_u32(screens[k]._pixels[hw_r][hw_c]) << 8u);
            }
        }
    }
}

void ws2812_write_screen_pixel(uint screen, uint row, uint col, rgb_t rgb) {
    if (screens[screen].row_col_flipped_before_inversion) {
        int hw_r = screens[screen].row_inverted ? NUM_ROWS-1 - col : col;
        int hw_c = screens[screen].col_inverted ? NUM_COLS-1 - row : row;
        screens[screen]._pixels[hw_r][hw_c] = rgb;
    } else {
        int hw_r = screens[screen].row_inverted ? NUM_ROWS-1 - row : row;
        int hw_c = screens[screen].col_inverted ? NUM_COLS-1 - col : col;
        screens[screen]._pixels[hw_r][hw_c] = rgb;
    }
}

void ws2812_blank_screen(uint screen) {
    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = 0; c < NUM_COLS; c++) {
            // r, c inversion doesnt matter here, all _pixels are set to black
            screens[screen]._pixels[r][c] = hex2rgb_t_f(0);
        }
    }
}

static inline uint32_t rgb_t2grb_u32(rgb_t pixel) {
    return
        ((uint32_t) (pixel.r) << 8) |
        ((uint32_t) (pixel.g) << 16) |
        (uint32_t) (pixel.b);
}
