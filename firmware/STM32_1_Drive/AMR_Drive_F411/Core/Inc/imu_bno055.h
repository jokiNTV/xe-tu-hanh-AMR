#ifndef IMU_BNO055_H
#define IMU_BNO055_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define BNO055_GRAVITY_M_S2                  9.80665f
#define BNO055_CHIP_ID_EXPECTED               0xA0U

/*
 * Default axis map from Bosch. This is intentionally configurable because
 * the exact board orientation on the AMR is mechanical-installation specific.
 * Change only after a physical +90/-90 test on the finished robot.
 */
#ifndef BNO055_AXIS_MAP_CONFIG_VALUE
#define BNO055_AXIS_MAP_CONFIG_VALUE          0x24U
#endif

#ifndef BNO055_AXIS_MAP_SIGN_VALUE
#define BNO055_AXIS_MAP_SIGN_VALUE            0x00U
#endif

/*
 * IMUPLUS = accelerometer + gyroscope fusion, magnetometer NOT used.
 * This is deliberate for the AMR: motors, steel chassis and high-current
 * wiring can corrupt magnetic heading. NDOF should only be enabled later
 * after a dedicated magnetic-interference test.
 */
#define BNO055_OPERATION_MODE_IMUPLUS         0x08U

typedef struct
{
    I2C_HandleTypeDef *i2c;
    uint16_t device_address; /* HAL 8-bit address: 0x28<<1 or 0x29<<1 */
    uint8_t chip_id;
    uint8_t operation_mode;
} Bno055Device_t;

typedef struct
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t quaternion_raw[4]; /* W, X, Y, Z */
    int8_t temperature_raw;

    float accel_m_s2[3];
    float gyro_dps[3];
    float temperature_c;

    float quaternion[4];      /* W, X, Y, Z */
    float fused_yaw_rad;
    float quaternion_norm;

    uint8_t calibration_system;
    uint8_t calibration_gyro;
    uint8_t calibration_accel;
    uint8_t calibration_mag;
    uint8_t self_test_result;
    uint8_t system_status;
    uint8_t system_error;

    bool quaternion_valid;
} Bno055Sample_t;

bool Bno055_Init(Bno055Device_t *device,
                 I2C_HandleTypeDef *i2c);

bool Bno055_ReadSample(Bno055Device_t *device,
                       Bno055Sample_t *sample);

#endif /* IMU_BNO055_H */
