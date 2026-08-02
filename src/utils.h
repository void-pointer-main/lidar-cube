#ifndef UTILS_H
#define UTILS_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979f

// i2c0
#define SDA0 0 // SDA2 on the schematic
#define SCL0 1 // SCL2 on the schematic

//i2c1
#define SDA1 18
#define SCL1 19

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

inline int max(int a, int b) {
    return a > b ? a : b;
}

inline int min(int a, int b) {
    return a < b ? a : b;
}

static void my_assert(bool expression, const char *filename, int line) {
    if (!expression) {
        while(1) {
            printf("ASSERT FAIL: file %s, line %d\n", filename, line);
            sleep_ms(500);
        }
    }
}

inline rgb_t rgb2rgb_t_f(uint8_t r, uint8_t g, uint8_t b) {
    rgb_t tmp;
    tmp.r = r;
    tmp.g = g;
    tmp.b = b;
    return tmp;
}

inline rgb_t hex2rgb_t_f(uint32_t rgb) {
    rgb_t tmp;
    tmp.r = (rgb >> 16) & 0xFF;
    tmp.g = (rgb >> 8) & 0xFF;
    tmp.b = rgb & 0xFF;
    return tmp;
}

inline rgb_t hex2rgb_t_f_modified_intensity(uint32_t rgb, uint mult, uint div) {
    rgb_t tmp;
    tmp.r = ((rgb >> 16) & 0xFF)*mult/div;
    tmp.g = ((rgb >> 8) & 0xFF)*mult/div;
    tmp.b = (rgb & 0xFF)*mult/div;
    return tmp;
}

inline rgb_t rgb_modified_intensity(rgb_t rgb, uint mult, uint div) {
    rgb.r = rgb.r * mult / div;
    rgb.g = rgb.g * mult / div;
    rgb.b = rgb.b * mult / div;
    return rgb;
}

rgb_t hsv2rgb_t_f(float h, float s, float v);

// uses hsluv
rgb_t distance_to_rgb_t_f(float distance);

// only for the terminal
void hsv_to_rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b);
inline uint8_t rgb_to_ansi256(uint8_t r, uint8_t g, uint8_t b)
{
    // map to 6x6x6 color cube (0–5 each channel)
    uint8_t ri = r / 51;
    uint8_t gi = g / 51;
    uint8_t bi = b / 51;

    return (uint8_t)(16 + 36 * ri + 6 * gi + bi);
}
uint8_t mm_to_color_id(int16_t mm);

void i2c_test(i2c_inst_t *i2c);

void shift_into_buffer(void *buffer, size_t len, size_t element_size, void *new_element);
float dot_product(const float vec1[3], const float vec2[3]);
float magnitude(float vec[3]);
void normalize(float vec[3]);
void scalar_mult(float vec[3], float scalar);

#endif
