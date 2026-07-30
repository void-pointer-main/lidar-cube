#ifndef ISM_HELPER_H
#define ISM_HELPER_H

#include "pico/stdlib.h"

void ism_init();
int ism_sample(float acc[3], float gyro[3]);

#endif
