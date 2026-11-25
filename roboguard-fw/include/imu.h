#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "mik32_hal_i2c.h"

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

} imu_sample_t;

typedef struct
{
    I2C_HandleTypeDef *i2c;

    uint8_t address;
    uint8_t who_am_i;

} imu_t;

HAL_StatusTypeDef imu_init(
    imu_t *imu,
    I2C_HandleTypeDef *i2c
);

HAL_StatusTypeDef imu_read(
    imu_t *imu,
    imu_sample_t *sample
);

#endif
