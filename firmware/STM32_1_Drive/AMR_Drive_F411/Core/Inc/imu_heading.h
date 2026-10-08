#ifndef IMU_HEADING_H
#define IMU_HEADING_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_bno055.h"

/* Stable samples required AFTER BNO gyro calibration reaches level 3. */
#define IMU_HEADING_CALIBRATION_SAMPLES      300U

typedef struct
{
    uint32_t calibration_count;
    uint32_t calibration_reset_count;

    float gyro_sum_z_dps;
    float gyro_bias_z_dps;

    /* Median-of-3 impulse filter state. */
    float gyro_history_dps[3];
    uint8_t gyro_history_count;
    uint8_t gyro_history_index;

    /* Low-pass + zero-rate bias tracking. */
    float filtered_gyro_z_dps;
    uint32_t stationary_count;

    /* Relative yaw estimator. */
    float yaw_rad;
    float sensor_zero_yaw_rad;
    bool sensor_zero_valid;

    uint32_t rejected_orientation_count;
    bool ready;
} ImuHeadingState_t;

typedef struct
{
    uint32_t calibration_count;
    uint32_t calibration_reset_count;
    float gyro_bias_z_dps;
    float gyro_z_dps;
    float gyro_z_rad_s;
    float yaw_rad;
    bool ready;
} ImuHeadingOutput_t;

void ImuHeading_Init(ImuHeadingState_t *state);

void ImuHeading_Update(ImuHeadingState_t *state,
                       const Bno055Sample_t *sample,
                       float period_s,
                       ImuHeadingOutput_t *output);

/* Re-zero ONLY the relative yaw reference.
 *
 * This does NOT:
 *   - reset BNO055
 *   - reset STM32
 *   - erase gyro bias calibration
 *   - change READY state
 *   - modify ROS/USB/controller interfaces
 *
 * The current physical robot heading becomes exactly 0 rad.
 * Call only with a fresh valid BNO055 sample.
 */
bool ImuHeading_ZeroYaw(ImuHeadingState_t *state,
                        const Bno055Sample_t *sample,
                        ImuHeadingOutput_t *output);

#endif /* IMU_HEADING_H */
