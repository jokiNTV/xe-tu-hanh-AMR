#ifndef DRIVE_ODOMETRY_H
#define DRIVE_ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Odometry 2D cho robot vi sai:
 * - Encoder: quang duong tinh tien.
 * - MPU6050: thay doi huong yaw.
 * He toa do: x tien, y trai, yaw duong khi quay trai (CCW).
 */

typedef struct
{
    bool initialized;
    bool valid;
    float x_m;
    float y_m;
    float yaw_rad;
    float previous_left_position_m;
    float previous_right_position_m;
    float previous_imu_yaw_rad;
    uint32_t previous_timestamp_ms;
    uint32_t sample_count;
    uint32_t reset_count;
    uint32_t rejected_sample_count;
} DriveOdometryState_t;

typedef struct
{
    bool valid;
    float x_m;
    float y_m;
    float yaw_rad;
    float linear_velocity_m_s;
    float angular_velocity_rad_s;
    uint32_t timestamp_ms;
    uint32_t sample_count;
} DriveOdometryOutput_t;

void DriveOdometry_Init(DriveOdometryState_t *state);

void DriveOdometry_Reset(DriveOdometryState_t *state,
                         float left_position_m,
                         float right_position_m,
                         float imu_yaw_rad,
                         uint32_t timestamp_ms);

bool DriveOdometry_Update(DriveOdometryState_t *state,
                          bool imu_ready,
                          float left_position_m,
                          float right_position_m,
                          float left_speed_m_s,
                          float right_speed_m_s,
                          float imu_yaw_rad,
                          float imu_gyro_z_rad_s,
                          uint32_t timestamp_ms,
                          DriveOdometryOutput_t *output);

#endif
