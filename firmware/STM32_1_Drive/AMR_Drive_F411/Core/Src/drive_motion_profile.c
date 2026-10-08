#include "drive_motion_profile.h"

#include <stddef.h>

#define DRIVE_MOTION_PROFILE_PI_RAD 3.14159265358979323846f

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static float MinFloat(float first, float second)
{
    return (first < second) ? first : second;
}

static float WrapAngleRad(float angle_rad)
{
    while (angle_rad > DRIVE_MOTION_PROFILE_PI_RAD)
    {
        angle_rad -= 2.0f * DRIVE_MOTION_PROFILE_PI_RAD;
    }

    while (angle_rad < -DRIVE_MOTION_PROFILE_PI_RAD)
    {
        angle_rad += 2.0f * DRIVE_MOTION_PROFILE_PI_RAD;
    }

    return angle_rad;
}

static bool IsFiniteImuMeasurement(float yaw_rad, float gyro_z_rad_s)
{
    /* Comparisons reject NaN as well. */
    return (yaw_rad >= -DRIVE_MOTION_PROFILE_PI_RAD) &&
           (yaw_rad <= DRIVE_MOTION_PROFILE_PI_RAD) &&
           (gyro_z_rad_s >= -20.0f) &&
           (gyro_z_rad_s <= 20.0f);
}

static bool IsStraightCommand(float left_rad_s, float right_rad_s)
{
    const bool same_direction =
        ((left_rad_s > 0.0f) && (right_rad_s > 0.0f)) ||
        ((left_rad_s < 0.0f) && (right_rad_s < 0.0f));

    return same_direction &&
           (AbsFloat(left_rad_s) >= DRIVE_MOTION_PROFILE_MIN_WHEEL_RAD_S) &&
           (AbsFloat(right_rad_s) >= DRIVE_MOTION_PROFILE_MIN_WHEEL_RAD_S) &&
           (AbsFloat(left_rad_s - right_rad_s) <=
            DRIVE_MOTION_PROFILE_STRAIGHT_DIFF_RAD_S);
}

static float SignFloat(float value)
{
    return (value < 0.0f) ? -1.0f : 1.0f;
}

static float ComputeAlignTarget(float left_rad_s, float right_rad_s)
{
    const float minimum_requested_magnitude =
        MinFloat(AbsFloat(left_rad_s), AbsFloat(right_rad_s));
    const float align_magnitude =
        MinFloat(DRIVE_MOTION_PROFILE_ALIGN_WHEEL_RAD_S,
                 minimum_requested_magnitude);

    return SignFloat(left_rad_s + right_rad_s) * align_magnitude;
}

static float StepJerkLimited(float current_speed_rad_s,
                             float target_speed_rad_s,
                             float *current_accel_rad_s2,
                             float period_s)
{
    const float speed_error = target_speed_rad_s - current_speed_rad_s;

    if (AbsFloat(speed_error) <= DRIVE_MOTION_PROFILE_SPEED_DONE_RAD_S)
    {
        *current_accel_rad_s2 = 0.0f;
        return target_speed_rad_s;
    }

    const bool accelerating_magnitude =
        AbsFloat(target_speed_rad_s) > AbsFloat(current_speed_rad_s);
    const float accel_limit = accelerating_magnitude ?
        DRIVE_MOTION_PROFILE_MAX_ACCEL_RAD_S2 :
        DRIVE_MOTION_PROFILE_MAX_DECEL_RAD_S2;
    const float requested_accel_rad_s2 =
        (speed_error > 0.0f) ? accel_limit : -accel_limit;
    const float maximum_accel_change_rad_s2 =
        DRIVE_MOTION_PROFILE_MAX_JERK_RAD_S3 * period_s;

    *current_accel_rad_s2 +=
        ClampFloat(requested_accel_rad_s2 - *current_accel_rad_s2,
                   -maximum_accel_change_rad_s2,
                   maximum_accel_change_rad_s2);

    float next_speed_rad_s =
        current_speed_rad_s + (*current_accel_rad_s2 * period_s);

    /* Never overshoot the requested wheel target. */
    if (((speed_error > 0.0f) && (next_speed_rad_s > target_speed_rad_s)) ||
        ((speed_error < 0.0f) && (next_speed_rad_s < target_speed_rad_s)))
    {
        next_speed_rad_s = target_speed_rad_s;
        *current_accel_rad_s2 = 0.0f;
    }

    return next_speed_rad_s;
}

static void FillOutput(const DriveMotionProfileState_t *state,
                       float left_target_rad_s,
                       float right_target_rad_s,
                       float yaw_error_rad,
                       uint32_t now_ms,
                       DriveMotionProfileOutput_t *output)
{
    output->left_target_rad_s = left_target_rad_s;
    output->right_target_rad_s = right_target_rad_s;
    output->reference_yaw_rad = state->reference_yaw_rad;
    output->yaw_error_rad = yaw_error_rad;
    output->elapsed_ms = now_ms - state->start_ms;
    output->stable_ms = state->stable_timer_active ?
        (now_ms - state->stable_since_ms) : 0U;
    output->status = (uint8_t)state->status;
    output->active = (state->status != DRIVE_MOTION_PROFILE_IDLE) &&
                     (state->status != DRIVE_MOTION_PROFILE_RUN);
    output->fault = state->fault_latched;
}

void DriveMotionProfile_Init(DriveMotionProfileState_t *state)
{
    DriveMotionProfile_Reset(state);
}

void DriveMotionProfile_Reset(DriveMotionProfileState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->status = DRIVE_MOTION_PROFILE_IDLE;
    state->reference_yaw_rad = 0.0f;
    state->profiled_left_rad_s = 0.0f;
    state->profiled_right_rad_s = 0.0f;
    state->left_accel_rad_s2 = 0.0f;
    state->right_accel_rad_s2 = 0.0f;
    state->start_ms = 0U;
    state->stable_since_ms = 0U;
    state->stable_timer_active = false;
    state->fault_latched = false;
}

void DriveMotionProfile_Update(DriveMotionProfileState_t *state,
                               bool imu_ready,
                               float yaw_rad,
                               float gyro_z_rad_s,
                               float left_command_rad_s,
                               float right_command_rad_s,
                               float period_s,
                               uint32_t now_ms,
                               DriveMotionProfileOutput_t *output)
{
    if ((state == NULL) || (output == NULL))
    {
        return;
    }

#if !DRIVE_MOTION_PROFILE_ENABLE
    DriveMotionProfile_Reset(state);
    output->left_target_rad_s = left_command_rad_s;
    output->right_target_rad_s = right_command_rad_s;
    output->reference_yaw_rad = 0.0f;
    output->yaw_error_rad = 0.0f;
    output->elapsed_ms = 0U;
    output->stable_ms = 0U;
    output->status = DRIVE_MOTION_PROFILE_IDLE;
    output->active = false;
    output->fault = false;
    return;
#else
    const bool valid_period = (period_s > 0.0f) && (period_s <= 1.0f);
    const bool valid_imu = imu_ready &&
        IsFiniteImuMeasurement(yaw_rad, gyro_z_rad_s);
    const bool straight_command =
        IsStraightCommand(left_command_rad_s, right_command_rad_s);

    if (!valid_period || !valid_imu || !straight_command)
    {
        DriveMotionProfile_Reset(state);
        output->left_target_rad_s = left_command_rad_s;
        output->right_target_rad_s = right_command_rad_s;
        output->reference_yaw_rad = 0.0f;
        output->yaw_error_rad = 0.0f;
        output->elapsed_ms = 0U;
        output->stable_ms = 0U;
        output->status = DRIVE_MOTION_PROFILE_IDLE;
        output->active = false;
        output->fault = false;
        return;
    }

    const float direction = SignFloat(left_command_rad_s + right_command_rad_s);

    /* A direction reversal is a new caster event; restart from ALIGN. */
    if ((state->status != DRIVE_MOTION_PROFILE_IDLE) &&
        (state->status != DRIVE_MOTION_PROFILE_FAULT_TIMEOUT) &&
        ((state->profiled_left_rad_s * direction) < 0.0f))
    {
        DriveMotionProfile_Reset(state);
    }

    if (state->status == DRIVE_MOTION_PROFILE_IDLE)
    {
        const float align_target_rad_s =
            ComputeAlignTarget(left_command_rad_s, right_command_rad_s);

        state->status = DRIVE_MOTION_PROFILE_ALIGN;
        state->reference_yaw_rad = yaw_rad;
        state->profiled_left_rad_s = align_target_rad_s;
        state->profiled_right_rad_s = align_target_rad_s;
        state->left_accel_rad_s2 = 0.0f;
        state->right_accel_rad_s2 = 0.0f;
        state->start_ms = now_ms;
        state->stable_since_ms = 0U;
        state->stable_timer_active = false;
        state->fault_latched = false;
    }

    const float yaw_error_rad =
        WrapAngleRad(state->reference_yaw_rad - yaw_rad);
    const uint32_t elapsed_ms = now_ms - state->start_ms;
    const bool chassis_stable =
        (AbsFloat(yaw_error_rad) <= DRIVE_MOTION_PROFILE_YAW_TOLERANCE_RAD) &&
        (AbsFloat(gyro_z_rad_s) <=
         DRIVE_MOTION_PROFILE_GYRO_TOLERANCE_RAD_S);

    if (((state->status == DRIVE_MOTION_PROFILE_ALIGN) ||
         (state->status == DRIVE_MOTION_PROFILE_STABILIZE)) &&
        (elapsed_ms >= DRIVE_MOTION_PROFILE_ALIGN_TIMEOUT_MS))
    {
        state->status = DRIVE_MOTION_PROFILE_FAULT_TIMEOUT;
        state->fault_latched = true;
        state->profiled_left_rad_s = 0.0f;
        state->profiled_right_rad_s = 0.0f;
        state->left_accel_rad_s2 = 0.0f;
        state->right_accel_rad_s2 = 0.0f;
    }

    switch (state->status)
    {
        case DRIVE_MOTION_PROFILE_ALIGN:
        {
            const float align_target_rad_s =
                ComputeAlignTarget(left_command_rad_s, right_command_rad_s);
            state->profiled_left_rad_s = align_target_rad_s;
            state->profiled_right_rad_s = align_target_rad_s;

            if (chassis_stable)
            {
                if (!state->stable_timer_active)
                {
                    state->stable_since_ms = now_ms;
                    state->stable_timer_active = true;
                }
            }
            else
            {
                state->stable_timer_active = false;
                state->stable_since_ms = 0U;
            }

            if (elapsed_ms >= DRIVE_MOTION_PROFILE_ALIGN_MIN_TIME_MS)
            {
                const uint32_t stable_ms = state->stable_timer_active ?
                    (now_ms - state->stable_since_ms) : 0U;

                if (stable_ms >= DRIVE_MOTION_PROFILE_STABLE_TIME_MS)
                {
                    state->status = DRIVE_MOTION_PROFILE_RAMP;
                    state->left_accel_rad_s2 = 0.0f;
                    state->right_accel_rad_s2 = 0.0f;
                }
                else
                {
                    state->status = DRIVE_MOTION_PROFILE_STABILIZE;
                }
            }
            break;
        }

        case DRIVE_MOTION_PROFILE_STABILIZE:
        {
            const float align_target_rad_s =
                ComputeAlignTarget(left_command_rad_s, right_command_rad_s);
            state->profiled_left_rad_s = align_target_rad_s;
            state->profiled_right_rad_s = align_target_rad_s;

            if (chassis_stable)
            {
                if (!state->stable_timer_active)
                {
                    state->stable_since_ms = now_ms;
                    state->stable_timer_active = true;
                }

                if ((now_ms - state->stable_since_ms) >=
                    DRIVE_MOTION_PROFILE_STABLE_TIME_MS)
                {
                    state->status = DRIVE_MOTION_PROFILE_RAMP;
                    state->left_accel_rad_s2 = 0.0f;
                    state->right_accel_rad_s2 = 0.0f;
                }
            }
            else
            {
                state->stable_timer_active = false;
                state->stable_since_ms = 0U;
            }
            break;
        }

        case DRIVE_MOTION_PROFILE_RAMP:
        {
            state->profiled_left_rad_s =
                StepJerkLimited(state->profiled_left_rad_s,
                                left_command_rad_s,
                                &state->left_accel_rad_s2,
                                period_s);
            state->profiled_right_rad_s =
                StepJerkLimited(state->profiled_right_rad_s,
                                right_command_rad_s,
                                &state->right_accel_rad_s2,
                                period_s);

            if ((AbsFloat(state->profiled_left_rad_s - left_command_rad_s) <=
                 DRIVE_MOTION_PROFILE_SPEED_DONE_RAD_S) &&
                (AbsFloat(state->profiled_right_rad_s - right_command_rad_s) <=
                 DRIVE_MOTION_PROFILE_SPEED_DONE_RAD_S))
            {
                state->profiled_left_rad_s = left_command_rad_s;
                state->profiled_right_rad_s = right_command_rad_s;
                state->left_accel_rad_s2 = 0.0f;
                state->right_accel_rad_s2 = 0.0f;
                state->status = DRIVE_MOTION_PROFILE_RUN;
            }
            break;
        }

        case DRIVE_MOTION_PROFILE_RUN:
            state->profiled_left_rad_s = left_command_rad_s;
            state->profiled_right_rad_s = right_command_rad_s;
            break;

        case DRIVE_MOTION_PROFILE_FAULT_TIMEOUT:
            state->profiled_left_rad_s = 0.0f;
            state->profiled_right_rad_s = 0.0f;
            break;

        case DRIVE_MOTION_PROFILE_IDLE:
        default:
            DriveMotionProfile_Reset(state);
            state->profiled_left_rad_s = left_command_rad_s;
            state->profiled_right_rad_s = right_command_rad_s;
            break;
    }

    FillOutput(state,
               state->profiled_left_rad_s,
               state->profiled_right_rad_s,
               yaw_error_rad,
               now_ms,
               output);
#endif
}
