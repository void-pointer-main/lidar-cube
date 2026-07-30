#include "PCF8575_helper.h"

#include "hardware/i2c.h"

#define I2C_HW i2c1
#define PCF_ADDR 0x27

static uint16_t current_mask = 0;

void PCF_set_pin(int pin, bool high) {
    uint16_t mask = current_mask;
    if (high) mask |= (1 << pin);
    else mask &= (0xFFFF & (0 << pin));
    PCF_set_mask(mask);
}

void PCF_set_mask(uint16_t mask) {
    i2c_set_baudrate(I2C_HW, 400 * 1000); // set up I2C frequency

    uint8_t buf[2] = {mask & 0xFF, mask >> 8};
    i2c_write_blocking(I2C_HW, PCF_ADDR, buf, 2, false);
    
    current_mask = mask;

    i2c_set_baudrate(I2C_HW, 1000 * 1000);
}