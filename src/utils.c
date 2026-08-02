#include "utils.h"

#include "hsluv.h"
#include <string.h>

uint8_t mm_to_color_id(int16_t mm)
{
    if (mm <= 0) return 196;   // strong red
    if (mm >= 2000) return 21; // strong blue

    float t = (float)mm / 200.0f;

    // hue: red (0°) -> blue (240°)
    float hue = (1.0f - t) * 0.0f + t * 240.0f;

    uint8_t r, g, b;
    hsv_to_rgb(hue, 1.0f, 1.0f, &r, &g, &b);

    return rgb_to_ansi256(r, g, b);
}

rgb_t hsv2rgb_t_f(float h, float s, float v) {
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

rgb_t distance_to_rgb_t_f(float distance) {
    if (distance < 0.0f) distance = 0.0f;
    if (distance > 1.0f) distance = 1.0f;

    // HSLuv
    float h = (distance) * 260.0;
    float s = 100.0;
    float l = 45.0;

    float r_tmp, g_tmp, b_tmp;
    hsluv2rgb(h, s, l, &r_tmp, &g_tmp, &b_tmp);

    rgb_t out = {
        .r = (uint8_t)(r_tmp * 255.0 + 0.5),
        .g = (uint8_t)(g_tmp * 255.0 + 0.5),
        .b = (uint8_t)(b_tmp * 255.0 + 0.5)
    };

    return out;
}

// only for the terminal

void hsv_to_rgb(float h, float s, float v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
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

    *r = (uint8_t)((rp + m) * 255);
    *g = (uint8_t)((gp + m) * 255);
    *b = (uint8_t)((bp + m) * 255);
}

static bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void i2c_test(i2c_inst_t *i2c) {
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
}

void shift_into_buffer(void *buffer, size_t len, size_t element_size, void *new_element) {
    char *buf = (char *)buffer;

    memmove(buf + element_size, buf, (len-1) * element_size);
    memcpy(buf, new_element, element_size);
}

float dot_product(const float vec1[3], const float vec2[3]) {
    return vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2]; 
}

float magnitude(float vec[3]) {
    return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

void normalize(float vec[3]) {
    float l = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
    scalar_mult(vec, 1/magnitude(vec));
}

void scalar_mult(float vec[3], float scalar) {
    for (int i = 0; i < 3; i++) {
        vec[i] *= scalar;
    }
}
