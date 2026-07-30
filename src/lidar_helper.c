#include "lidar_helper.h"

#include <stdio.h>

#include "vl53l7cx_api.h"
#include "PCF8575_helper.h"
#include "utils.h"

#define RANGING_FREQUENCY_HZ 18
#define SHARPENER_PERCENTAGE 15
#define TEMP_CALIBRATION_LOOP_CNT 1000
#define INTEGRATION_TIME_MS 3

#define DEFAULT_VL53L7CX_ADDR 0x29

#define INT1 11
#define INT2 12
#define INT3 13
#define INT4 14
#define INT5 15
#define INT6 16
#define INT7 17
#define INT8 20
#define INT9 21
#define INT10 22
#define INT11 2
#define INT12 3
const int interrupt_pins[NUM_LIDARS] = {INT1, INT2, INT3, INT4, INT5, INT6, INT7, INT8, INT9, INT10, INT11, INT12};
// surely there is a better way to do a 'hash'?
static inline int pin_to_lidar_index(int pin) {
    switch (pin) {
        case INT1:
            return 0;
        case INT2:
            return 1;
        case INT3:
            return 2;
        case INT4:
            return 3;
        case INT5:
            return 4;
        case INT6:
            return 5;
        case INT7:
            return 6;
        case INT8:
            return 7;
        case INT9:
            return 8;
        case INT10:
            return 9;
        case INT11:
            return 10;
        case INT12:
            return 11;
        default:
            return -1;
    }
}

typedef struct {
    VL53L7CX_Configuration dev;
    volatile bool data_ready;
    uint32_t array[256];
} lidar_t;

lidar_t lidars[NUM_LIDARS];

const bool ignore_sensor_flag[NUM_LIDARS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void data_ready_handler(uint gpio, uint32_t event_mask);
static inline bool range_valid(int range_status) {
    return range_status == 5 ||
            range_status == 9 ||
            range_status == 10 ||
            range_status == 12;
}

int lidars_init() {

    uint8_t status = 0;

    for (int i = 0; i < NUM_LIDARS/2; i++) {
        lidars[i].dev.platform.i2c_inst = i2c0;
    }

    for (int i = NUM_LIDARS/2; i < NUM_LIDARS; i++) {
        lidars[i].dev.platform.i2c_inst = i2c1;
    }

    PCF_set_mask(0x0000);
    for (int i = 0; i < NUM_LIDARS; i++) {

        if (ignore_sensor_flag[i]) continue;

        lidars[i].data_ready = false;

        VL53L7CX_Configuration *lid_ptr = &(lidars[i].dev); // just for readability

        // enable irq handler so that we can register a data ready interrupt
        gpio_init(interrupt_pins[i]);
        gpio_set_dir(interrupt_pins[i], false);
        gpio_pull_up(interrupt_pins[i]);
        gpio_set_irq_callback(data_ready_handler);
        gpio_set_irq_enabled(interrupt_pins[i], GPIO_IRQ_EDGE_FALL, true);
        
        PCF_set_pin(i, 1);

        lid_ptr->platform.address = DEFAULT_VL53L7CX_ADDR;

        uint8_t is_alive;

        status = vl53l7cx_is_alive(lid_ptr, &is_alive);
        if(!is_alive || status)
        {
            // we test in case the address has already been set.
            lid_ptr->platform.address = DEFAULT_VL53L7CX_ADDR + i%(NUM_LIDARS/2);
            status = vl53l7cx_is_alive(lid_ptr, &is_alive);
            my_assert(is_alive && !status, __FILE__, __LINE__);
        }

        status = vl53l7cx_init(lid_ptr);
        my_assert(!status, __FILE__, __LINE__);


        status = vl53l7cx_set_i2c_address(lid_ptr, DEFAULT_VL53L7CX_ADDR + i%(NUM_LIDARS/2));
        my_assert(!status, __FILE__, __LINE__);

        status = vl53l7cx_set_ranging_mode(lid_ptr, VL53L7CX_RANGING_MODE_AUTONOMOUS);
        my_assert(!status, __FILE__, __LINE__);

        status = vl53l7cx_set_integration_time_ms(lid_ptr, INTEGRATION_TIME_MS);
        my_assert(!status, __FILE__, __LINE__);

        status = vl53l7cx_set_ranging_frequency_hz(lid_ptr, RANGING_FREQUENCY_HZ);
        my_assert(!status, __FILE__, __LINE__);

        status = vl53l7cx_set_resolution(lid_ptr, VL53L7CX_RESOLUTION_8X8);
        my_assert(!status, __FILE__, __LINE__);

        status = vl53l7cx_set_VHV_repeat_count(lid_ptr, TEMP_CALIBRATION_LOOP_CNT);
        my_assert(!status, __FILE__, __LINE__);

        status = vl53l7cx_set_sharpener_percent(lid_ptr, SHARPENER_PERCENTAGE);
        my_assert(!status, __FILE__, __LINE__);

        PCF_set_pin(i, 0);
    }

    for (int i = 0; i < NUM_LIDARS; i++) {
        if (ignore_sensor_flag[i]) continue;
        PCF_set_pin(i, 1);
    }

    // enable the data ready handler irq
    irq_set_enabled(IO_IRQ_BANK0, true);

end:
    return status;
}

void lidars_start_sampling() {
    uint32_t previous_time = time_us_32();
    for (int i = 0; i < NUM_LIDARS; i++) {
        if (ignore_sensor_flag[i]) continue;

        vl53l7cx_start_ranging(&(lidars[i].dev));
        // while (time_us_32() - previous_time < 49859);
        // previous_time = time_us_32();
    }
}

/* we check continously through the lidars for ones with available data */
void lidars_sample(int16_t results[NUM_LIDARS][VLX_NUM_ROWS][VLX_NUM_COLS]) {

    VL53L7CX_ResultsData res;

    int num_lidars_dealt_with = 0;
    bool lidar_read[NUM_LIDARS] = {0};

    while (num_lidars_dealt_with < NUM_LIDARS) {
        for (int i = 0; i < NUM_LIDARS; i++) {
            if (lidar_read[i]) {
                continue;
            }
            if (ignore_sensor_flag[i]) {
                for (int r = 0; r < VLX_NUM_ROWS; r++) {
                    for (int c = 0; c < VLX_NUM_COLS; c++) {
                        results[i][r][c] = MAX_DIST_MM;
                    }
                }
                lidar_read[i] = true;
                num_lidars_dealt_with++;
                continue;
            }
            if (!(lidars[i].data_ready)) {
                continue;
            }

            lidars[i].data_ready = false;
            lidar_read[i] = true;
            num_lidars_dealt_with++;
            vl53l7cx_get_ranging_data(&(lidars[i].dev), &res);
            for (int r = 0; r < VLX_NUM_ROWS; r++) {
                for (int c = 0; c < VLX_NUM_COLS; c++) {
                    int index = (r)*8 + (7-c); // we flip the image vertically

                    if (!range_valid(res.target_status[index])) {
                        results[i][r][c] = MAX_DIST_MM;
                    } else {
                        results[i][r][c] = res.distance_mm[index];
                    }
                }
            }
        }
    }
}

void data_ready_handler(uint gpio, uint32_t event_mask) {
    int index = pin_to_lidar_index(gpio);
    if (index < 0) return;
    lidars[index].data_ready = true;
}
