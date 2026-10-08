#include "imu_mpu6050.h"

#include <stddef.h>

#define MPU6050_ADDRESS_AD0_LOW          (0x68U << 1)
#define MPU6050_ADDRESS_AD0_HIGH         (0x69U << 1)

#define MPU6050_REG_SMPLRT_DIV           0x19U
#define MPU6050_REG_CONFIG               0x1AU
#define MPU6050_REG_GYRO_CONFIG          0x1BU
#define MPU6050_REG_ACCEL_CONFIG         0x1CU
#define MPU6050_REG_ACCEL_XOUT_H         0x3BU
#define MPU6050_REG_PWR_MGMT_1           0x6BU
#define MPU6050_REG_WHO_AM_I             0x75U

#define MPU6050_WHO_AM_I_VALUE           0x68U
#define MPU6050_I2C_TIMEOUT_MS           20U

/* +/-4 g va +/-500 degree/s. */
#define MPU6050_ACCEL_LSB_PER_G           8192.0f
#define MPU6050_GYRO_LSB_PER_DPS          65.5f

static bool WriteRegister(Mpu6050Device_t *device,
                          uint8_t reg,
                          uint8_t value)
{
    return HAL_I2C_Mem_Write(device->i2c,
                             device->device_address,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             MPU6050_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool ReadRegisters(Mpu6050Device_t *device,
                          uint8_t reg,
                          uint8_t *data,
                          uint16_t length)
{
    return HAL_I2C_Mem_Read(device->i2c,
                            device->device_address,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            length,
                            MPU6050_I2C_TIMEOUT_MS) == HAL_OK;
}

static int16_t ReadSignedBigEndian(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) |
                     (uint16_t)data[1]);
}

static bool DetectAddress(Mpu6050Device_t *device)
{
    const uint16_t candidates[] =
    {
        MPU6050_ADDRESS_AD0_LOW,
        MPU6050_ADDRESS_AD0_HIGH
    };

    for (uint32_t index = 0U;
         index < (sizeof(candidates) / sizeof(candidates[0]));
         ++index)
    {
        uint8_t who_am_i = 0U;

        device->device_address = candidates[index];

        if (HAL_I2C_IsDeviceReady(device->i2c,
                                  device->device_address,
                                  2U,
                                  MPU6050_I2C_TIMEOUT_MS) != HAL_OK)
        {
            continue;
        }

        if (!ReadRegisters(device,
                           MPU6050_REG_WHO_AM_I,
                           &who_am_i,
                           1U))
        {
            continue;
        }

        if (who_am_i == MPU6050_WHO_AM_I_VALUE)
        {
            device->who_am_i = who_am_i;
            return true;
        }
    }

    return false;
}

bool Mpu6050_Init(Mpu6050Device_t *device,
                  I2C_HandleTypeDef *i2c)
{
    if ((device == NULL) || (i2c == NULL))
    {
        return false;
    }

    device->i2c = i2c;
    device->device_address = 0U;
    device->who_am_i = 0U;

    if (!DetectAddress(device))
    {
        return false;
    }

    /* Reset, sau do chon PLL X gyro lam clock. */
    if (!WriteRegister(device, MPU6050_REG_PWR_MGMT_1, 0x80U))
    {
        return false;
    }

    HAL_Delay(100U);

    if (!WriteRegister(device, MPU6050_REG_PWR_MGMT_1, 0x01U))
    {
        return false;
    }

    /* DLPF 44 Hz, sample 100 Hz, gyro +/-500 dps, accel +/-4 g. */
    if (!WriteRegister(device, MPU6050_REG_CONFIG, 0x03U) ||
        !WriteRegister(device, MPU6050_REG_SMPLRT_DIV, 9U) ||
        !WriteRegister(device, MPU6050_REG_GYRO_CONFIG, 0x08U) ||
        !WriteRegister(device, MPU6050_REG_ACCEL_CONFIG, 0x08U))
    {
        return false;
    }

    HAL_Delay(20U);
    return true;
}

bool Mpu6050_ReadSample(Mpu6050Device_t *device,
                        Mpu6050Sample_t *sample)
{
    uint8_t frame[14];

    if ((device == NULL) ||
        (device->i2c == NULL) ||
        (sample == NULL))
    {
        return false;
    }

    if (!ReadRegisters(device,
                       MPU6050_REG_ACCEL_XOUT_H,
                       frame,
                       sizeof(frame)))
    {
        return false;
    }

    sample->accel_raw[0] = ReadSignedBigEndian(&frame[0]);
    sample->accel_raw[1] = ReadSignedBigEndian(&frame[2]);
    sample->accel_raw[2] = ReadSignedBigEndian(&frame[4]);
    sample->temperature_raw = ReadSignedBigEndian(&frame[6]);
    sample->gyro_raw[0] = ReadSignedBigEndian(&frame[8]);
    sample->gyro_raw[1] = ReadSignedBigEndian(&frame[10]);
    sample->gyro_raw[2] = ReadSignedBigEndian(&frame[12]);

    for (uint32_t axis = 0U; axis < 3U; ++axis)
    {
        sample->accel_m_s2[axis] =
            ((float)sample->accel_raw[axis] /
             MPU6050_ACCEL_LSB_PER_G) * MPU6050_GRAVITY_M_S2;

        sample->gyro_dps[axis] =
            (float)sample->gyro_raw[axis] /
            MPU6050_GYRO_LSB_PER_DPS;
    }

    sample->temperature_c =
        ((float)sample->temperature_raw / 340.0f) + 36.53f;

    return true;
}
