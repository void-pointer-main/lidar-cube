#include <stdio.h>
#include "pico/stdlib.h"

#include "utils.h"
#include <stdio.h>
#include "lidar_helper.h"
#include "hardware/i2c.h"
#include "display_helper.h"
#include "ism_helper.h"
#include "lidar_rejection_helper.h"
#include "ws2812_helper.h"
#include "mode_state_machine.h"
#include "screen_saver.h"

#include "gol_engine.h"
#include "membrane_engine.h"

#include <stdint.h>
#include <math.h>

typedef enum {
    PROJECTION,
    GOL,
    RIPPLE,
    LOW_POWER,
    NUM_MODES,
} mode_t;

mode_t lidar_cube_mode;

void init_i2c();

void mode_init(mode_t mode);
void mode_release(mode_t mode);

int main() {
    stdio_init_all();

    init_i2c();

    ism_init();

    rejection_helper_init();
    mode_state_machine_init();

    displays_init();
    
    uint32_t st = time_us_32();
    my_assert(lidars_init() == 0, __FILE__, __LINE__);
    uint32_t et = time_us_32();
    printf("lidar init time: %d us\n", et-st);
    
    lidars_start_sampling();

    printf("init success\n");

    float acc[3] = {0}, gyro[3] = {0};

    lidar_cube_mode = RIPPLE;
    mode_init(lidar_cube_mode);

    while (1) {
        ism_sample(acc, gyro);

        // printf("%.2f,%.2f,%.2f\n", acc[0], acc[1], acc[2]);

        if (mode_state_machine_update(gyro)) {
            mode_release(lidar_cube_mode);
            lidar_cube_mode = (lidar_cube_mode + 1) % NUM_MODES;
            mode_init(lidar_cube_mode);
        }

        bool moving;
        int dir;
        rejection_helper_update(acc, &moving, &dir);

        int16_t results_mm[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS] = {0};
        int16_t collective_pixel_dists[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS] = {0};
        if (lidar_cube_mode != LOW_POWER) {
            lidars_sample(results_mm);
            displays_project(results_mm, !moving, dir);
            displays_get_collective_pixel_dists(collective_pixel_dists);
        } else {
            sleep_ms(30);
        }

        
        switch (lidar_cube_mode) {
            case PROJECTION:
                displays_distance_to_color_write();
                break;

            case GOL:
                gol_update_and_write(collective_pixel_dists);
                break;

            case RIPPLE:
                membrane_update_and_write(collective_pixel_dists);
                break;

            case LOW_POWER:
                screen_saver_update();
                break;
            
            default:
                break;
        }
        uint32_t et = time_us_32();
        
        // printf("%d\n", et-st);

        ws2812_display_screens();
    }
}

void mode_init(mode_t mode) {
    switch (mode) {
        case PROJECTION:
            break;

        case GOL:
            gol_init();
            break;

        case RIPPLE:
            membrane_init();
            break;

        case LOW_POWER:
            lidars_pause_sampling();
            screen_saver_init();
            break;
        
        default:
            break;
    }
}

void mode_release(mode_t mode) {
    switch (mode) {
        case PROJECTION:
            break;

        case GOL:
            gol_release();
            break;

        case RIPPLE:
            membrane_release();
            break;

        case LOW_POWER:
            screen_saver_release();
            for (int s = 0; s < NUM_SCREENS; s++) {
                ws2812_blank_screen(s);
            }
            ws2812_display_screens();
            lidars_start_sampling();
            break;

        default:
            break;
    }
}

void init_i2c() {
    i2c_init(i2c0, 1000 * 1000);
    gpio_set_function(SDA0, GPIO_FUNC_I2C);
    gpio_set_function(SCL0, GPIO_FUNC_I2C);
    gpio_pull_up(SDA0);
    gpio_pull_up(SCL0);

    i2c_init(i2c1, 1000 * 1000); // PCF8575 uses 400 KHz, see the helper files.
    gpio_set_function(SDA1, GPIO_FUNC_I2C);
    gpio_set_function(SCL1, GPIO_FUNC_I2C);
    gpio_pull_up(SDA1);
    gpio_pull_up(SCL1);
}
