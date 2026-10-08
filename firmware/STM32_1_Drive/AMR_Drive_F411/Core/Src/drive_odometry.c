#include "drive_odometry.h"

#include <math.h>
#include <stddef.h>

#define DRIVE_ODOM_PI_RAD                  (3.14159265358979323846f)
#define DRIVE_ODOM_TWO_PI_RAD              (6.28318530717958647692f)
#define DRIVE_ODOM_MAX_WHEEL_DELTA_M       (0.100f)
#define DRIVE_ODOM_MAX_IMU_DELTA_RAD       (0.500f)
#define DRIVE_ODOM_MAX_SAMPLE_PERIOD_MS    (100U)
#define DRIVE_ODOM_MAX_ABS_POSITION_M      (100000.0f)
#define DRIVE_ODOM_MAX_ABS_SPEED_M_S       (10.0f)
#define DRIVE_ODOM_MAX_ABS_GYRO_RAD_S      (20.0f)
#define DRIVE_ODOM_WHEEL_SEPARATION_M      (0.600f)

static float WrapAngle(float angle_rad)
{
    while (angle_rad > DRIVE_ODOM_PI_RAD)
    {
        angle_rad -= DRIVE_ODOM_TWO_PI_RAD;
    }

    while (angle_rad < -DRIVE_ODOM_PI_RAD)
    {
        angle_rad += DRIVE_ODOM_TWO_PI_RAD;
    }

    return angle_rad;
}

static bool IsFiniteInRange(float value, float absolute_limit)
{
    return isfinite(value) && (fabsf(value) <= absolute_limit);
}

static void FillOutput(const DriveOdometryState_t *state,
                       float linear_velocity_m_s,
                       float angular_velocity_rad_s,
                       uint32_t timestamp_ms,
                       DriveOdometryOutput_t *output)
{
    output->valid = state->valid;
    output->x_m = state->x_m;
    output->y_m = state->y_m;
    output->yaw_rad = state->yaw_rad;
    output->linear_velocity_m_s = linear_velocity_m_s;
    output->angular_velocity_rad_s = angular_velocity_rad_s;
    output->timestamp_ms = timestamp_ms;
    output->sample_count = state->sample_count;
}

void DriveOdometry_Init(DriveOdometryState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    *state = (DriveOdometryState_t){0};
}

void DriveOdometry_Reset(DriveOdometryState_t *state,
                         float left_position_m,
                         float right_position_m,
                         float imu_yaw_rad,
                         uint32_t timestamp_ms)
{
    if (state == NULL)
    {
        return;
    }

    const uint32_t next_reset_count = state->reset_count + 1U;

    *state = (DriveOdometryState_t){0};
    state->previous_left_position_m = left_position_m;
    state->previous_right_position_m = right_position_m;
    state->previous_imu_yaw_rad = WrapAngle(imu_yaw_rad);
    state->previous_timestamp_ms = timestamp_ms;
    state->initialized = true;
    state->valid = true;
    state->reset_count = next_reset_count;
}

bool DriveOdometry_Update(DriveOdometryState_t *state,
                          bool imu_ready,
                          float left_position_m,
                          float right_position_m,
                          float left_speed_m_s,
                          float right_speed_m_s,
                          float imu_yaw_rad,
                          float imu_gyro_z_rad_s,
                          uint32_t timestamp_ms,
                          DriveOdometryOutput_t *output)
{
    if ((state == NULL) || (output == NULL))
    {
        return false;
    }

    const bool input_valid =
        imu_ready &&
        IsFiniteInRange(left_position_m, DRIVE_ODOM_MAX_ABS_POSITION_M) &&
        IsFiniteInRange(right_position_m, DRIVE_ODOM_MAX_ABS_POSITION_M) &&
        IsFiniteInRange(left_speed_m_s, DRIVE_ODOM_MAX_ABS_SPEED_M_S) &&
        IsFiniteInRange(right_speed_m_s, DRIVE_ODOM_MAX_ABS_SPEED_M_S) &&
        IsFiniteInRange(imu_yaw_rad, DRIVE_ODOM_PI_RAD + 0.01f) &&
        IsFiniteInRange(imu_gyro_z_rad_s,
                        DRIVE_ODOM_MAX_ABS_GYRO_RAD_S);

    if (!input_valid)
    {
        state->initialized = false;
        state->valid = false;
        ++state->rejected_sample_count;
        FillOutput(state, 0.0f, 0.0f, timestamp_ms, output);
        return false;
    }

    if (!state->initialized)
    {
        state->previous_left_position_m = left_position_m;
        state->previous_right_position_m = right_position_m;
        state->previous_imu_yaw_rad = WrapAngle(imu_yaw_rad);
        state->previous_timestamp_ms = timestamp_ms;
        state->initialized = true;
        state->valid = true;
        ++state->sample_count;

        FillOutput(state,
                   0.5f * (left_speed_m_s + right_speed_m_s),
                   (right_speed_m_s - left_speed_m_s) /
                       DRIVE_ODOM_WHEEL_SEPARATION_M,
                   timestamp_ms,
                   output);
        return true;
    }

    const uint32_t sample_period_ms =
        timestamp_ms - state->previous_timestamp_ms;
    const float left_delta_m =
        left_position_m - state->previous_left_position_m;
    const float right_delta_m =
        right_position_m - state->previous_right_position_m;

    const float wheel_yaw_delta_rad =
        (right_delta_m - left_delta_m) /
        DRIVE_ODOM_WHEEL_SEPARATION_M;

    const bool delta_valid =
        (sample_period_ms > 0U) &&
        (sample_period_ms <= DRIVE_ODOM_MAX_SAMPLE_PERIOD_MS) &&
        (fabsf(left_delta_m) <= DRIVE_ODOM_MAX_WHEEL_DELTA_M) &&
        (fabsf(right_delta_m) <= DRIVE_ODOM_MAX_WHEEL_DELTA_M) &&
        (fabsf(wheel_yaw_delta_rad) <= DRIVE_ODOM_MAX_IMU_DELTA_RAD);

    if (!delta_valid)
    {
        /* Re-baseline de lan sau khong cong don mot buoc nhay sai. */
        state->previous_left_position_m = left_position_m;
        state->previous_right_position_m = right_position_m;
        state->previous_imu_yaw_rad = WrapAngle(imu_yaw_rad);
        state->previous_timestamp_ms = timestamp_ms;
        state->valid = false;
        ++state->rejected_sample_count;
        FillOutput(state, 0.0f, 0.0f, timestamp_ms, output);
        return false;
    }

    const float distance_delta_m =
        0.5f * (left_delta_m + right_delta_m);
    const float midpoint_yaw_rad =
        WrapAngle(state->yaw_rad + (0.5f * wheel_yaw_delta_rad));

    state->x_m += distance_delta_m * cosf(midpoint_yaw_rad);
    state->y_m += distance_delta_m * sinf(midpoint_yaw_rad);
    state->yaw_rad =
        WrapAngle(state->yaw_rad + wheel_yaw_delta_rad);

    state->previous_left_position_m = left_position_m;
    state->previous_right_position_m = right_position_m;
    state->previous_imu_yaw_rad = WrapAngle(imu_yaw_rad);
    state->previous_timestamp_ms = timestamp_ms;
    state->valid = true;
    ++state->sample_count;

    FillOutput(state,
               0.5f * (left_speed_m_s + right_speed_m_s),
               (right_speed_m_s - left_speed_m_s) /
                   DRIVE_ODOM_WHEEL_SEPARATION_M,
               timestamp_ms,
               output);
    return true;
}
