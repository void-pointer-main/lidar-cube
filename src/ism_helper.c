#include "ism_helper.h"

#include "hardware/i2c.h"
#include "ISM330IS-PID/ism330is_reg.h"
#include "utils.h"

// Functions defined for STs driver
#define ISM_I2C_HW i2c0
#define ISM_I2C_ADDR 0x6A

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {  
    int ret = i2c_write_burst_blocking(ISM_I2C_HW, ISM_I2C_ADDR, &reg, 1);
    if (ret == PICO_ERROR_GENERIC) {
        return -1;
    }
    if (ret != 1) {
        return -2;
    }

    ret = i2c_write_blocking(ISM_I2C_HW, ISM_I2C_ADDR, bufp, len, false);
    if (ret == PICO_ERROR_GENERIC) {
        return -1;
    }
    if (ret != len) {
        return -3;
    }

    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    int ret = i2c_write_burst_blocking(ISM_I2C_HW, ISM_I2C_ADDR, &reg, 1);
    if (ret == PICO_ERROR_GENERIC) {
        return -1;
    }
    if (ret != 1) {
        return -2;
    }

    ret = i2c_read_blocking(ISM_I2C_HW, ISM_I2C_ADDR, bufp, len, false);
    if (ret == PICO_ERROR_GENERIC) {
        return -1;
    }
    if (ret != len) {
        return -3;
    }

    return 0;
}
//////////////////////////////////////

#define ACC_LSB_2G 0.061f
#define ACC_LSB_4G 0.122f
#define ACC_LSB_8G 0.244f
#define ACC_LSB_16G 0.488f
#define GYRO_LSB_125DPS 4.375f
#define GYRO_LSB_250DPS 8.75f
#define GYRO_LSB_500DPS 17.5f
#define GYRO_LSB_1000DPS 35.f
#define GYRO_LSB_2000DPS 70.f

#define ACC_ODR ISM330IS_XL_ODR_AT_208Hz_LP
#define GYRO_ODR ISM330IS_GY_ODR_AT_208Hz_LP
#define ACC_FS ISM330IS_2g
#define GYRO_FS ISM330IS_500dps
#define ACC_LSB ACC_LSB_2G
#define GYRO_LSB GYRO_LSB_500DPS

#define NUM_CALIBRATION_CYCLES 500

static int16_t _raw_acc[3];
static int16_t _raw_gyro[3];

static int16_t _acc_calibration[3] = {0};
static int16_t _gyro_calibration[3] = {0};

static stmdev_ctx_t dev_ctx;

// see https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/ism330is_STdC/examples/ism330is_read_data_polling.c
void ism_init() {
    /* Initialize mems driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = sleep_ms;

    uint8_t whoamI = 0;
    int ret = ism330is_device_id_get(&dev_ctx, &whoamI);

    my_assert(whoamI == ISM330IS_ID, __FILE__, __LINE__);
    
    /* Restore default configuration */
    ism330is_software_reset(&dev_ctx);
    
    /* Set Output Data Rate */
    ism330is_xl_data_rate_set(&dev_ctx, ACC_ODR);
    ism330is_gy_data_rate_set(&dev_ctx, GYRO_ODR);
    
    /* Set full scale */
    ism330is_xl_full_scale_set(&dev_ctx, ACC_FS);
    ism330is_gy_full_scale_set(&dev_ctx, GYRO_FS);

    ism_calibrate_gyro();
}

void ism_calibrate_gyro() {
    float gyro_sum[3] = {0};
    for (int i = 0; i < NUM_CALIBRATION_CYCLES; i++) {
        float acc[3], gyro[3];
        ism_sample(acc, gyro);
        for (int j = 0; j < 3; j++) {
            gyro_sum[j] += gyro[j];
        }
        sleep_ms(5);
    }

    for (int j = 0; j < 3; j++) {
        _gyro_calibration[j] = gyro_sum[j] / NUM_CALIBRATION_CYCLES;
    }
}

int ism_sample(float acc[3], float gyro[3]) {
    uint8_t drdy = 0;
    ism330is_xl_flag_data_ready_get(&dev_ctx, &drdy);
    if (drdy) {
        ism330is_acceleration_raw_get(&dev_ctx, _raw_acc);
        for (int i = 0; i < 3; i++) {
            acc[i] = (_raw_acc[i]) * ACC_LSB - _acc_calibration[i];
        }
    }

    ism330is_gy_flag_data_ready_get(&dev_ctx, &drdy);
    if (drdy) {
        ism330is_angular_rate_raw_get(&dev_ctx, _raw_gyro);
        for (int i = 0; i < 3; i++) {
            gyro[i] = (_raw_gyro[i]) * GYRO_LSB - _gyro_calibration[i];
        }
    }

    return drdy;
}
