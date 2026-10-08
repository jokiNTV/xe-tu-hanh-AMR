#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define MPU6050_GRAVITY_M_S2 (9.80665f)

typedef struct
{
    I2C_HandleTypeDef *i2c;
    uint16_t device_address;
    uint8_t who_am_i;
} Mpu6050Device_t;

typedef struct
{
    int16_t accel_raw[3];
    int16_t temperature_raw;
    int16_t gyro_raw[3];

    float accel_m_s2[3];
    float temperature_c;
    float gyro_dps[3];
} Mpu6050Sample_t;

bool Mpu6050_Init(Mpu6050Device_t *device,
                  I2C_HandleTypeDef *i2c);

bool Mpu6050_ReadSample(Mpu6050Device_t *device,
                        Mpu6050Sample_t *sample);

#endif /* IMU_MPU6050_H */
