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

static float display_vectors[NUM_SCREENS][3];

void rejection_helper_init() {
    movement_state = MOVING;
}

void rejection_helper_update(float acc[3], bool *moving, int *rejected_display) {
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
