#ifndef PCF8575_HELPER_H
#define PCF8575_HELPER_H

#include "pico/stdlib.h"

void PCF_set_pin(int pin, bool high);
void PCF_set_mask(uint16_t mask);

#endif
