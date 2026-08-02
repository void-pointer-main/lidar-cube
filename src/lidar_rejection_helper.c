#include "lidar_rejection_helper.h"

#include "utils.h"
#include "ws2812_helper.h"

#define MOVEMENT_DIFF_MAGNITUDE_THRESHOLD_mG 40.f
#define MOVEMENT_TIMEOUT_US (2000 * 1000)

static enum movement_states_t {
    NOT_MOVING,
    MOVING,
    MOVING_COUNTDOWN,
    NUM_MOVEMENT_STATES
} movement_state;

const float display_vectors[NUM_SCREENS][3] = {
    {0, 0, 1.f},
    {0, 0, -1.f},
    {0, -1.f, 0},
    {0, 1.f, 0},
    {1.f, 0, 0},
    {-1.f, 0, 0}
};

void rejection_helper_init() {
    movement_state = MOVING;
}

void rejection_helper_update(float acc[3], bool *moving, int *rejected_direction) {
    static float prev_acc[3] = {0};

    uint64_t current_time_us = time_us_64();
    static uint64_t countdown_start_time_us = 0;

    *moving = true;

    float diff[3];
    for (int i = 0; i < 3; i++) {
        diff[i] = acc[i] - prev_acc[i];
        prev_acc[i] = acc[i];
    }
    float diff_mag = magnitude(diff);

    float max_abs_dp = 0;
    int max_index = 0;

    for (int s = 0; s < NUM_SCREENS; s++) {
        for (int i = 0; i < 3; i++) {
            float dp = dot_product(display_vectors[s], acc);
            if (dp > max_abs_dp) {
                max_abs_dp = dp;
                max_index = s;
            }
        }
    }
    *rejected_direction = max_index;

    switch (movement_state) {
        case NOT_MOVING:

            if (diff_mag > MOVEMENT_DIFF_MAGNITUDE_THRESHOLD_mG) { 
                movement_state = MOVING;
            }

            *moving = false;
            break;
        case MOVING:

            if (diff_mag < MOVEMENT_DIFF_MAGNITUDE_THRESHOLD_mG) {
                countdown_start_time_us = current_time_us;
                movement_state = MOVING_COUNTDOWN;
            }

            *moving = true;
            break;
        case MOVING_COUNTDOWN:

            if (diff_mag > MOVEMENT_DIFF_MAGNITUDE_THRESHOLD_mG) {
                movement_state = MOVING;
            }
            else if (current_time_us - countdown_start_time_us > MOVEMENT_TIMEOUT_US) {
                movement_state = NOT_MOVING;
            }

            *moving = true;
            break;
        default:
            break;
    }

}
