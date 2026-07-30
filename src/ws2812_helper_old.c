/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Basically just a repackaging of the ws2812 pi pico example */

#include "ws2812_helper.h"
#include <math.h>

screen_t screens[NUM_SCREENS];

#define PANEL_DISTANCE_FROM_CENTER 3.6f
#define PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL 2.8f
#define PIXEL2PIXEL_INCREMENT (2.8f*2/NUM_ROWS)

void ws2812_init() {
    uint offset_0 = pio_add_program(pio0, &ws2812_program);
    if (offset_0 == PICO_ERROR_GENERIC){
        while(1) printf("pio0 load fail\n");
    }

    uint offset_1 = pio_add_program(pio1, &ws2812_program);
    if (offset_1 == PICO_ERROR_GENERIC){
        while(1) printf("pio0 load fail\n");
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

    screens[TOP].row_inverted = true;
    screens[BOTTOM].row_inverted = true;
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

    /* Setting up the pixel vectors */

    int screen_num = FRONT;

    float x_rc_is_00 = PANEL_DISTANCE_FROM_CENTER;
    float x_increment_with_r = 0.0;
    float x_increment_with_c = 0.0;

    float y_rc_is_00 = -PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    float y_increment_with_r = 0.0;
    float y_increment_with_c = PIXEL2PIXEL_INCREMENT;

    float z_rc_is_00 = PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    float z_increment_with_r = -PIXEL2PIXEL_INCREMENT;
    float z_increment_with_c = 0.0;

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = 0; c < NUM_ROWS; c++) {
            screens[screen_num].pixel_vectors[r][c][0] = x_rc_is_00 + x_increment_with_r*r + x_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][1] = y_rc_is_00 + y_increment_with_r*r + y_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][2] = z_rc_is_00 + z_increment_with_r*r + z_increment_with_c*c;
        }
    }

    screen_num = RIGHT;

    x_rc_is_00 = PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    x_increment_with_r = 0.0;
    x_increment_with_c = -PIXEL2PIXEL_INCREMENT;

    y_rc_is_00 = PANEL_DISTANCE_FROM_CENTER;
    y_increment_with_r = 0.0;
    y_increment_with_c = 0.0;

    z_rc_is_00 = PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    z_increment_with_r = -PIXEL2PIXEL_INCREMENT;
    z_increment_with_c = 0.0;

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = 0; c < NUM_ROWS; c++) {
            screens[screen_num].pixel_vectors[r][c][0] = x_rc_is_00 + x_increment_with_r*r + x_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][1] = y_rc_is_00 + y_increment_with_r*r + y_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][2] = z_rc_is_00 + z_increment_with_r*r + z_increment_with_c*c;
        }
    }

    screen_num = BACK;

    x_rc_is_00 = -PANEL_DISTANCE_FROM_CENTER;
    x_increment_with_r = 0.0;
    x_increment_with_c = 0.0;

    y_rc_is_00 = PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    y_increment_with_r = 0.0;
    y_increment_with_c = -PIXEL2PIXEL_INCREMENT;

    z_rc_is_00 = PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    z_increment_with_r = -PIXEL2PIXEL_INCREMENT;
    z_increment_with_c = 0.0;

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = 0; c < NUM_ROWS; c++) {
            screens[screen_num].pixel_vectors[r][c][0] = x_rc_is_00 + x_increment_with_r*r + x_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][1] = y_rc_is_00 + y_increment_with_r*r + y_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][2] = z_rc_is_00 + z_increment_with_r*r + z_increment_with_c*c;
        }
    }

    screen_num = LEFT;

    x_rc_is_00 = -PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    x_increment_with_r = 0.0;
    x_increment_with_c = PIXEL2PIXEL_INCREMENT;

    y_rc_is_00 = -PANEL_DISTANCE_FROM_CENTER;
    y_increment_with_r = 0.0;
    y_increment_with_c = 0.0;

    z_rc_is_00 = PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    z_increment_with_r = -PIXEL2PIXEL_INCREMENT;
    z_increment_with_c = 0.0;

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = 0; c < NUM_ROWS; c++) {
            screens[screen_num].pixel_vectors[r][c][0] = x_rc_is_00 + x_increment_with_r*r + x_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][1] = y_rc_is_00 + y_increment_with_r*r + y_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][2] = z_rc_is_00 + z_increment_with_r*r + z_increment_with_c*c;
        }
    }

    screen_num = TOP;

    x_rc_is_00 = -PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    x_increment_with_r = PIXEL2PIXEL_INCREMENT;
    x_increment_with_c = 0.0;

    y_rc_is_00 = -PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    y_increment_with_r = 0.0;
    y_increment_with_c = PIXEL2PIXEL_INCREMENT;

    z_rc_is_00 = PANEL_DISTANCE_FROM_CENTER;
    z_increment_with_r = 0.0;
    z_increment_with_c = 0.0;

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = 0; c < NUM_ROWS; c++) {
            screens[screen_num].pixel_vectors[r][c][0] = x_rc_is_00 + x_increment_with_r*r + x_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][1] = y_rc_is_00 + y_increment_with_r*r + y_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][2] = z_rc_is_00 + z_increment_with_r*r + z_increment_with_c*c;
        }
    }

    screen_num = BOTTOM;

    x_rc_is_00 = PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    x_increment_with_r = -PIXEL2PIXEL_INCREMENT;
    x_increment_with_c = 0.0;

    y_rc_is_00 = -PANEL_DISTANCE_CENTER_TO_OUTER_PIXEL;
    y_increment_with_r = 0.0;
    y_increment_with_c = PIXEL2PIXEL_INCREMENT;

    z_rc_is_00 = -PANEL_DISTANCE_FROM_CENTER;
    z_increment_with_r = 0.0;
    z_increment_with_c = 0.0;

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = 0; c < NUM_ROWS; c++) {
            screens[screen_num].pixel_vectors[r][c][0] = x_rc_is_00 + x_increment_with_r*r + x_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][1] = y_rc_is_00 + y_increment_with_r*r + y_increment_with_c*c;
            screens[screen_num].pixel_vectors[r][c][2] = z_rc_is_00 + z_increment_with_r*r + z_increment_with_c*c;
        }
    }

    for (int i = 0; i < NUM_SCREENS; i++) {
        ws2812_program_init(screens[i].pio, screens[i].sm, screens[i].offset, screens[i].pin, 800000, IS_RGBW);
    }
}

void ws2812_display_screens() {
    for (int hw_r = 0; hw_r < NUM_ROWS; hw_r++) {
        for (int hw_c = 0; hw_c < NUM_COLS; hw_c++) {
            for (int k = 0; k < NUM_SCREENS; k++) {
                pio_sm_put_blocking(screens[k].pio, screens[k].sm, rgb_t2grb_u32(screens[k].pixels[hw_r][hw_c]) << 8u);
            }
        }
    }
}

void ws2812_write_screen_pixel(uint screen, uint row, uint col, rgb_t rgb) {
    int hw_r = screens[screen].row_inverted ? NUM_ROWS-1 - row : row;
    int hw_c = screens[screen].col_inverted ? NUM_COLS-1 - col : col;
    screens[screen].pixels[hw_r][hw_c] = rgb;
}

void ws2812_blank_screen(uint screen) {
    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = 0; c < NUM_COLS; c++) {
            // r, c inversion doesnt matter here, all pixels are set to black
            screens[screen].pixels[r][c] = hex2rgb_t_f(0);
        }
    }
}

static inline uint32_t rgb_t2grb_u32(rgb_t pixel) {
    return
        ((uint32_t) (pixel.r) << 8) |
        ((uint32_t) (pixel.g) << 16) |
        (uint32_t) (pixel.b);
}

rgb_t hsv2rgb_t_f(float h, float s, float v)
{
    float c = v * s;
    float x = c * (1.0f - __builtin_fabsf(fmodf(h / 60.0f, 2) - 1));
    float m = v - c;

    float rp, gp, bp;

    if      (h < 60)  { rp = c; gp = x; bp = 0; }
    else if (h < 120) { rp = x; gp = c; bp = 0; }
    else if (h < 180) { rp = 0; gp = c; bp = x; }
    else if (h < 240) { rp = 0; gp = x; bp = c; }
    else if (h < 300) { rp = x; gp = 0; bp = c; }
    else               { rp = c; gp = 0; bp = x; }

    rgb_t tmp;

    tmp.r = (uint8_t)((rp + m) * 255);
    tmp.g = (uint8_t)((gp + m) * 255);
    tmp.b = (uint8_t)((bp + m) * 255);

    return tmp;
}