#include <stdio.h>
#include "pico/stdlib.h"

#include "utils.h"
#include <stdio.h>
#include "lidar_helper.h"
#include "hardware/i2c.h"
#include "display_helper.h"
#include "ism_helper.h"
#include "lidar_rejection_helper.h"

#include <stdint.h>
#include <math.h>

void init_i2c();

int main()
{
    stdio_init_all();

    init_i2c();

    ism_init();

    rejection_helper_init();

    displays_init();
    my_assert(lidars_init() == 0, __FILE__, __LINE__);
    lidars_start_sampling();

    printf("init success\n");

    float acc[3] = {0}, gyro[3] = {0};

    while (1) {
        int16_t results_mm[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS] = {0};

        uint32_t st = time_us_32();
        lidars_sample(results_mm);
        uint32_t et = time_us_32();
        
        int drdy = ism_sample(acc, gyro);

        bool moving;
        int d;
        rejection_helper_update(acc, &moving, &d);

        printf("\x1b[1;1H");
        for (int l = 0; l < 6; l++) {
            for (int r = 0; r < 8; r++) {
                for (int c = 0; c < 8; c++) {
                    printf("\x1b[48;5;%dm  \x1b[0;0m", mm_to_color_id(results_mm[l][r][c]));
                    // printf("%4d ", results_mm[l][r][c]);             
                }
                putchar('\n');
            }
            putchar('\n');
        }
        putchar('\n');
        printf("%d\n", et-st);

        displays_update(results_mm);
        // sleep_ms(60);
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
