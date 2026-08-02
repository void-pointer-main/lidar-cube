#include "mode_state_machine.h"

#include "utils.h"
#include <string.h>

#define MAG_THRESHOLD 400000.f
#define FIRST_PEAK_DEADTIME_US (200 * 1000)
#define SECOND_PEAK_TIMEOUT_US (350 * 1000)
#define SECOND_PEAK_DEADTIME_US (200 * 1000)

typedef enum {
    WAITING,
    PEAK_2,
} mode_state_machine_state_t;

mode_state_machine_state_t mode_state_machine_state;

void mode_state_machine_init() {
    mode_state_machine_state = WAITING;
}

bool mode_state_machine_update(float gyro[3]) {
    bool mode_change = false;

    uint64_t current_time_us = time_us_64();
    static uint64_t state_switch_time_us = 0;

    static float prev_peak[3] = {0};

    float mag = magnitude(gyro);

    switch (mode_state_machine_state) {
    case WAITING:
        
        if (current_time_us - state_switch_time_us < SECOND_PEAK_DEADTIME_US) break;

        if (mag > MAG_THRESHOLD) {
            mode_state_machine_state = PEAK_2;
            state_switch_time_us = current_time_us;
            memcpy(prev_peak, gyro, sizeof(prev_peak));
            break;
        }

        break;

    case PEAK_2:
        
        if (current_time_us - state_switch_time_us < FIRST_PEAK_DEADTIME_US) break;

        if (current_time_us - state_switch_time_us > SECOND_PEAK_TIMEOUT_US) {
            mode_state_machine_state = WAITING;
            break;
        }

        if (mag > MAG_THRESHOLD && dot_product(prev_peak, gyro) > 0) {
            mode_change = true;
            mode_state_machine_state = WAITING;
            state_switch_time_us = current_time_us;
            break;
        }

        break;

    default:
        break;
    }

    return mode_change;
}