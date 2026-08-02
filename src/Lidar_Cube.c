#include <stdio.h>
#include "pico/stdlib.h"

#include "utils.h"
#include <stdio.h>
#include "lidar_helper.h"
#include "hardware/i2c.h"
#include "display_helper.h"
#include "ism_helper.h"
#include "lidar_rejection_helper.h"
// #include "PCF8575_helper.h"
#include "ws2812_helper.h"
#include "mode_state_machine.h"

#include "gol_engine.h"

#include <stdint.h>
#include <math.h>

typedef enum {
    PROJECTION,
    GOL,
    RIPPLE,
    NUM_MODES,
} mode_t;

mode_t lidar_cube_mode;

void init_i2c();

void mode_init(mode_t mode);
void mode_release(mode_t mode);

int main()
{
    stdio_init_all();

    init_i2c();

    ism_init();

    rejection_helper_init();
    mode_state_machine_init();

    displays_init();
    // my_assert(lidars_init() == 0, __FILE__, __LINE__);
    // lidars_start_sampling();

    // PCF_set_mask(0x00);

    printf("init success\n");

    float acc[3] = {0}, gyro[3] = {0};


    lidar_cube_mode = GOL;
    gol_init();

    while (1) {
        int16_t results_mm[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS] = {0};

        // uint32_t st = time_us_32();
        // lidars_sample(results_mm);
        // uint32_t et = time_us_32();
        
        ism_sample(acc, gyro);

        // printf("%.2f,%.2f,%.2f\n", gyro[0], gyro[1], gyro[2]);

        if (mode_state_machine_update(gyro)) {
            lidar_cube_mode = (lidar_cube_mode + 1) % NUM_MODES;
        }

        bool moving;
        int d;
        rejection_helper_update(acc, &moving, &d);

        displays_project(results_mm);
        int16_t collective_pixel_dists[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS];
        displays_get_collective_pixel_dists(collective_pixel_dists);

        switch (lidar_cube_mode) {
            case PROJECTION:
                
                break;

            case GOL:
                gol_update_and_write(collective_pixel_dists);
                break;

            case RIPPLE:
                
                break;
            
            default:
                break;
            }

        // ws2812_display_screens();

        sleep_ms(50);
    }
}

void mode_init(mode_t mode) {
    switch (mode)
    {
    case PROJECTION:
        
        break;

    case GOL:
        gol_init();
        break;

    case RIPPLE:
        
        break;
    
    default:
        break;
    }
}

void mode_release(mode_t mode) {
        switch (mode)
    {
    case PROJECTION:
        
        break;

    case GOL:
        gol_release();
        break;

    case RIPPLE:
        
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
