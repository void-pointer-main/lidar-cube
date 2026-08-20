#include "screen_saver.h"

#include "ws2812_helper.h"

#include <math.h>

static uint32_t image[8][8] = {
    { 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000 },
    { 0x000000, 0x000700, 0x009100, 0x00F100, 0x00F100, 0x009200, 0x000700, 0x000000 },
    { 0x000000, 0x009100, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x009300, 0x000000 },
    { 0x000000, 0x00F100, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00F200, 0x000000 },
    { 0x000000, 0x00F100, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x00F200, 0x000000 },
    { 0x000000, 0x009300, 0x00FF00, 0x00FF00, 0x00FF00, 0x00FF00, 0x009500, 0x000000 },
    { 0x000000, 0x000800, 0x009500, 0x00F200, 0x00F200, 0x009500, 0x000800, 0x000000 },
    { 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000 }
};

#define MAX_LOOP_DIV 3.f
#define INC 0.02f
#define BASE 0.0005f
#define COEF 4.f
#define BIAS 8.f
#define OFFSET 0.4f

static float loop_div = MAX_LOOP_DIV;
static float increment = -INC;

void screen_saver_init() {
    loop_div = MAX_LOOP_DIV;
    static float increment = -INC;
    for (int s = 0; s < NUM_SCREENS; s++) {
        ws2812_blank_screen(s);
    }
}

void screen_saver_release() {

}

void screen_saver_update() {

    if (loop_div > MAX_LOOP_DIV) {
        increment = -INC;
    }
    else if (loop_div < 0.f) {
        increment = INC;
    }

    loop_div += increment;

    for (int s = 0; s < NUM_SCREENS; s++) {
        for (int r = 0; r < NUM_ROWS; r++) {
            for (int c = 0; c < NUM_COLS; c++) {
                if (image[r][c] == 0) continue;
                ws2812_write_screen_pixel(s, r, c, rgb_modified_intensity(hex2rgb_t_f(image[r][c]), 1, (uint)(COEF*powf(BASE, -(loop_div-OFFSET))+BIAS)));
            }
        }
    }
}
