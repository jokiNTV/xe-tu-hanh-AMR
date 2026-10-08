#include "imu_bno055.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define BNO055_ADDRESS_A                     (0x28U << 1)
#define BNO055_ADDRESS_B                     (0x29U << 1)

#define BNO055_REG_CHIP_ID                   0x00U
#define BNO055_REG_PAGE_ID                   0x07U
#define BNO055_REG_ACCEL_DATA_X_LSB          0x08U
#define BNO055_REG_TEMP                      0x34U
#define BNO055_REG_CALIB_STAT                0x35U
#define BNO055_REG_ST_RESULT                 0x36U
#define BNO055_REG_SYS_STATUS                0x39U
#define BNO055_REG_SYS_ERR                   0x3AU
#define BNO055_REG_UNIT_SEL                  0x3BU
#define BNO055_REG_OPR_MODE                  0x3DU
#define BNO055_REG_PWR_MODE                  0x3EU
#define BNO055_REG_TEMP_SOURCE               0x40U
#define BNO055_REG_AXIS_MAP_CONFIG           0x41U
#define BNO055_REG_AXIS_MAP_SIGN             0x42U

#define BNO055_MODE_CONFIG                   0x00U
#define BNO055_POWER_NORMAL                  0x00U
#define BNO055_PAGE_0                        0x00U

/* Units used by the old AMR stack:
 * accel = m/s^2, gyro = degree/s, temperature = Celsius.
 * UNIT_SEL 0x00 selects exactly these physical units.
 */
#define BNO055_UNIT_SELECTION                0x00U
#define BNO055_TEMP_SOURCE_GYRO              0x01U

#define BNO055_I2C_TIMEOUT_MS                20U
#define BNO055_MODE_TO_CONFIG_DELAY_MS       25U
#define BNO055_CONFIG_TO_MODE_DELAY_MS       10U
#define BNO055_STARTUP_DELAY_MS              700U

#define BNO055_ACCEL_LSB_PER_M_S2            100.0f
#define BNO055_GYRO_LSB_PER_DPS              16.0f
#define BNO055_QUATERNION_LSB                16384.0f

#define BNO055_PI_RAD                        3.14159265358979323846f

/* One contiguous read from 0x08 through 0x27 gives:
 * accel, mag, gyro, Euler and quaternion. Only accel/gyro/quaternion are used.
 */
#define BNO055_MOTION_BLOCK_LENGTH           32U
#define BNO055_STATUS_BLOCK_LENGTH           7U /* 0x34..0x3A */

static bool WriteRegister(Bno055Device_t *device,
                          uint8_t reg,
                          uint8_t value)
{
    return HAL_I2C_Mem_Write(device->i2c,
                             device->device_address,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             BNO055_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool ReadRegisters(Bno055Device_t *device,
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
                            BNO055_I2C_TIMEOUT_MS) == HAL_OK;
}

static int16_t ReadSignedLittleEndian(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] |
                     ((uint16_t)data[1] << 8));
}

static bool DetectAddress(Bno055Device_t *device)
{
    const uint16_t candidates[] =
    {
        BNO055_ADDRESS_A,
        BNO055_ADDRESS_B
    };

    for (uint32_t index = 0U;
         index < (sizeof(candidates) / sizeof(candidates[0]));
         ++index)
    {
        uint8_t chip_id = 0U;

        device->device_address = candidates[index];

        if (HAL_I2C_IsDeviceReady(device->i2c,
                                  device->device_address,
                                  3U,
                                  BNO055_I2C_TIMEOUT_MS) != HAL_OK)
        {
            continue;
        }

        if (!ReadRegisters(device,
                           BNO055_REG_CHIP_ID,
                           &chip_id,
                           1U))
        {
            continue;
        }

        if (chip_id == BNO055_CHIP_ID_EXPECTED)
        {
            device->chip_id = chip_id;
            return true;
        }
    }

    return false;
}

static bool SetOperationMode(Bno055Device_t *device,
                             uint8_t mode)
{
    if (!WriteRegister(device, BNO055_REG_OPR_MODE, mode))
    {
        return false;
    }

    if (mode == BNO055_MODE_CONFIG)
    {
        /* Datasheet requires 19 ms from operation mode -> CONFIGMODE. */
        HAL_Delay(BNO055_MODE_TO_CONFIG_DELAY_MS);
    }
    else
    {
        /* Datasheet requires 7 ms from CONFIGMODE -> operation mode. */
        HAL_Delay(BNO055_CONFIG_TO_MODE_DELAY_MS);
    }

    device->operation_mode = mode;
    return true;
}

static float QuaternionToYawRad(const float quaternion[4])
{
    const float w = quaternion[0];
    const float x = quaternion[1];
    const float y = quaternion[2];
    const float z = quaternion[3];

    const float sin_yaw = 2.0f * ((w * z) + (x * y));
    const float cos_yaw = 1.0f - 2.0f * ((y * y) + (z * z));

    return atan2f(sin_yaw, cos_yaw);
}

bool Bno055_Init(Bno055Device_t *device,
                 I2C_HandleTypeDef *i2c)
{
    uint8_t readback = 0U;

    if ((device == NULL) || (i2c == NULL))
    {
        return false;
    }

    memset(device, 0, sizeof(*device));
    device->i2c = i2c;

    /* BNO055 requires a long power-on startup interval. */
    HAL_Delay(BNO055_STARTUP_DELAY_MS);

    if (!DetectAddress(device))
    {
        return false;
    }

    if (!SetOperationMode(device, BNO055_MODE_CONFIG))
    {
        return false;
    }

    if (!WriteRegister(device, BNO055_REG_PAGE_ID, BNO055_PAGE_0) ||
        !WriteRegister(device, BNO055_REG_PWR_MODE, BNO055_POWER_NORMAL) ||
        !WriteRegister(device, BNO055_REG_UNIT_SEL, BNO055_UNIT_SELECTION) ||
        !WriteRegister(device, BNO055_REG_TEMP_SOURCE,
                       BNO055_TEMP_SOURCE_GYRO) ||
        !WriteRegister(device, BNO055_REG_AXIS_MAP_CONFIG,
                       BNO055_AXIS_MAP_CONFIG_VALUE) ||
        !WriteRegister(device, BNO055_REG_AXIS_MAP_SIGN,
                       BNO055_AXIS_MAP_SIGN_VALUE))
    {
        return false;
    }

    /*
     * ROOT-LEVEL magnetic-noise protection:
     * IMUPLUS deliberately excludes the magnetometer from heading fusion.
     */
    if (!SetOperationMode(device, BNO055_OPERATION_MODE_IMUPLUS))
    {
        return false;
    }

    /* Verify that the requested fusion mode really latched. */
    if (!ReadRegisters(device,
                       BNO055_REG_OPR_MODE,
                       &readback,
                       1U) ||
        ((readback & 0x0FU) != BNO055_OPERATION_MODE_IMUPLUS))
    {
        return false;
    }

    /* Wait for the fusion engine to report RUNNING (SYS_STATUS = 5).
     * This keeps the existing FreeRTOS ready/fault semantics deterministic.
     */
    for (uint32_t attempt = 0U; attempt < 50U; ++attempt)
    {
        uint8_t system_status = 0U;
        uint8_t system_error = 0U;

        if (!ReadRegisters(device,
                           BNO055_REG_SYS_STATUS,
                           &system_status,
                           1U) ||
            !ReadRegisters(device,
                           BNO055_REG_SYS_ERR,
                           &system_error,
                           1U))
        {
            return false;
        }

        if (system_status == 5U)
        {
            return true;
        }

        if ((system_status == 1U) || (system_error != 0U))
        {
            return false;
        }

        HAL_Delay(20U);
    }

    return false;
}

bool Bno055_ReadSample(Bno055Device_t *device,
                       Bno055Sample_t *sample)
{
    uint8_t motion[BNO055_MOTION_BLOCK_LENGTH];
    uint8_t status[BNO055_STATUS_BLOCK_LENGTH];

    if ((device == NULL) ||
        (device->i2c == NULL) ||
        (sample == NULL))
    {
        return false;
    }

    if (!ReadRegisters(device,
                       BNO055_REG_ACCEL_DATA_X_LSB,
                       motion,
                       sizeof(motion)))
    {
        return false;
    }

    if (!ReadRegisters(device,
                       BNO055_REG_TEMP,
                       status,
                       sizeof(status)))
    {
        return false;
    }

    /* 0x08..0x0D accelerometer. */
    sample->accel_raw[0] = ReadSignedLittleEndian(&motion[0]);
    sample->accel_raw[1] = ReadSignedLittleEndian(&motion[2]);
    sample->accel_raw[2] = ReadSignedLittleEndian(&motion[4]);

    /* 0x14..0x19 gyroscope; offset 0x14 - 0x08 = 12. */
    sample->gyro_raw[0] = ReadSignedLittleEndian(&motion[12]);
    sample->gyro_raw[1] = ReadSignedLittleEndian(&motion[14]);
    sample->gyro_raw[2] = ReadSignedLittleEndian(&motion[16]);

    /* 0x20..0x27 quaternion; offset 0x20 - 0x08 = 24. */
    sample->quaternion_raw[0] = ReadSignedLittleEndian(&motion[24]);
    sample->quaternion_raw[1] = ReadSignedLittleEndian(&motion[26]);
    sample->quaternion_raw[2] = ReadSignedLittleEndian(&motion[28]);
    sample->quaternion_raw[3] = ReadSignedLittleEndian(&motion[30]);

    sample->temperature_raw = (int8_t)status[0];

    for (uint32_t axis = 0U; axis < 3U; ++axis)
    {
        sample->accel_m_s2[axis] =
            (float)sample->accel_raw[axis] /
            BNO055_ACCEL_LSB_PER_M_S2;

        sample->gyro_dps[axis] =
            (float)sample->gyro_raw[axis] /
            BNO055_GYRO_LSB_PER_DPS;
    }

    sample->temperature_c = (float)sample->temperature_raw;

    float quaternion_norm_squared = 0.0f;
    for (uint32_t index = 0U; index < 4U; ++index)
    {
        sample->quaternion[index] =
            (float)sample->quaternion_raw[index] /
            BNO055_QUATERNION_LSB;
        quaternion_norm_squared +=
            sample->quaternion[index] * sample->quaternion[index];
    }

    sample->quaternion_norm = sqrtf(quaternion_norm_squared);
    sample->quaternion_valid =
        isfinite(sample->quaternion_norm) &&
        (sample->quaternion_norm >= 0.85f) &&
        (sample->quaternion_norm <= 1.15f);

    if (sample->quaternion_valid)
    {
        sample->fused_yaw_rad =
            QuaternionToYawRad(sample->quaternion);
    }
    else
    {
        sample->fused_yaw_rad = 0.0f;
    }

    /* Status block 0x34..0x3A. */
    const uint8_t calib = status[1];
    sample->calibration_system = (calib >> 6) & 0x03U;
    sample->calibration_gyro = (calib >> 4) & 0x03U;
    sample->calibration_accel = (calib >> 2) & 0x03U;
    sample->calibration_mag = calib & 0x03U;
    sample->self_test_result = status[2];
    sample->system_status = status[5];
    sample->system_error = status[6];

    /* A hard BNO system error is treated exactly like an I2C read failure so
     * the existing FreeRTOS reinitialization logic can recover safely.
     */
    if ((sample->system_status != 5U) ||
        (sample->system_error != 0U))
    {
        return false;
    }

    return true;
}
