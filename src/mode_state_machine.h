#ifndef MODE_STATE_MACHINE_H
#define MODE_STATE_MACHINE_H

#include "pico/stdlib.h"

void mode_state_machine_init();
bool mode_state_machine_update(float gyro[3]);

#endif
