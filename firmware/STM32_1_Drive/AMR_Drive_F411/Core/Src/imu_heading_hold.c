#include "imu_heading_hold.h"

#include <stddef.h>

#define IMU_HEADING_HOLD_PI_RAD 3.14159265358979323846f

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

#if IMU_HEADING_HOLD_V2_ENABLE
static float MaxFloat(float first, float second)
{
    return (first > second) ? first : second;
}

static float MinFloat(float first, float second)
{
    return (first < second) ? first : second;
}
#endif

static float WrapAngleRad(float angle_rad)
{
    while (angle_rad > IMU_HEADING_HOLD_PI_RAD)
    {
        angle_rad -= 2.0f * IMU_HEADING_HOLD_PI_RAD;
    }

    while (angle_rad < -IMU_HEADING_HOLD_PI_RAD)
    {
        angle_rad += 2.0f * IMU_HEADING_HOLD_PI_RAD;
    }

    return angle_rad;
}

static bool IsStraightCommand(float left_rad_s, float right_rad_s)
{
    const bool same_direction =
        ((left_rad_s > 0.0f) && (right_rad_s > 0.0f)) ||
        ((left_rad_s < 0.0f) && (right_rad_s < 0.0f));

    return same_direction &&
           (AbsFloat(left_rad_s) >=
            IMU_HEADING_HOLD_MIN_WHEEL_RAD_S) &&
           (AbsFloat(right_rad_s) >=
            IMU_HEADING_HOLD_MIN_WHEEL_RAD_S) &&
           (AbsFloat(left_rad_s - right_rad_s) <=
            IMU_HEADING_HOLD_STRAIGHT_DIFF_RAD_S);
}

static bool ImuMeasurementIsValid(float yaw_rad,
                                  float gyro_z_rad_s)
{
    /* Cac phep so sanh nay cung loai NaN vi moi so sanh voi NaN deu false. */
    return (yaw_rad >= -IMU_HEADING_HOLD_PI_RAD) &&
           (yaw_rad <= IMU_HEADING_HOLD_PI_RAD) &&
           (gyro_z_rad_s >= -20.0f) &&
           (gyro_z_rad_s <= 20.0f);
}

static void FillPassThrough(float left_command_rad_s,
                            float right_command_rad_s,
                            ImuHeadingHoldOutput_t *output)
{
    output->reference_yaw_rad = 0.0f;
    output->error_rad = 0.0f;
    output->desired_yaw_rate_rad_s = 0.0f;
    output->yaw_rate_error_rad_s = 0.0f;
    output->rate_ff_correction_rad_s = 0.0f;
    output->rate_p_correction_rad_s = 0.0f;
    output->rate_i_correction_rad_s = 0.0f;
    output->correction_rad_s = 0.0f;
    output->left_target_rad_s = left_command_rad_s;
    output->right_target_rad_s = right_command_rad_s;
    output->controller_version =
        (IMU_HEADING_HOLD_V2_ENABLE != 0) ? 2U : 1U;
    output->correction_saturated = false;
    output->active = false;
}

void ImuHeadingHold_Init(ImuHeadingHoldState_t *state)
{
    ImuHeadingHold_Reset(state);
}

void ImuHeadingHold_Reset(ImuHeadingHoldState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->reference_yaw_rad = 0.0f;
    state->desired_yaw_rate_rad_s = 0.0f;
    state->yaw_rate_integral_correction_rad_s = 0.0f;
    state->correction_rad_s = 0.0f;
    state->active = false;
}

#if !IMU_HEADING_HOLD_V2_ENABLE
static void UpdateLegacyController(ImuHeadingHoldState_t *state,
                                   float yaw_rad,
                                   float gyro_z_rad_s,
                                   float left_command_rad_s,
                                   float right_command_rad_s,
                                   float wheel_target_limit_rad_s,
                                   float period_s,
                                   ImuHeadingHoldOutput_t *output)
{
    float error_rad =
        WrapAngleRad(state->reference_yaw_rad - yaw_rad);

    if (AbsFloat(error_rad) <
        IMU_HEADING_HOLD_LEGACY_DEADBAND_RAD)
    {
        error_rad = 0.0f;
    }

    const float raw_correction_rad_s =
        (IMU_HEADING_HOLD_KP * error_rad) -
        (IMU_HEADING_HOLD_KD * gyro_z_rad_s);

    const float requested_correction_rad_s =
        ClampFloat(raw_correction_rad_s,
                   -IMU_HEADING_HOLD_LEGACY_CORRECTION_RAD_S,
                   IMU_HEADING_HOLD_LEGACY_CORRECTION_RAD_S);

    const float maximum_change_rad_s =
        IMU_HEADING_HOLD_LEGACY_SLEW_RAD_S2 * period_s;

    state->correction_rad_s +=
        ClampFloat(requested_correction_rad_s -
                       state->correction_rad_s,
                   -maximum_change_rad_s,
                   maximum_change_rad_s);

    output->reference_yaw_rad = state->reference_yaw_rad;
    output->error_rad = error_rad;
    output->desired_yaw_rate_rad_s = 0.0f;
    output->yaw_rate_error_rad_s = 0.0f;
    output->rate_ff_correction_rad_s = 0.0f;
    output->rate_p_correction_rad_s = 0.0f;
    output->rate_i_correction_rad_s = 0.0f;
    output->correction_rad_s = state->correction_rad_s;
    output->left_target_rad_s =
        ClampFloat(left_command_rad_s - state->correction_rad_s,
                   -wheel_target_limit_rad_s,
                   wheel_target_limit_rad_s);
    output->right_target_rad_s =
        ClampFloat(right_command_rad_s + state->correction_rad_s,
                   -wheel_target_limit_rad_s,
                   wheel_target_limit_rad_s);
    output->controller_version = 1U;
    output->correction_saturated =
        raw_correction_rad_s != requested_correction_rad_s;
    output->active = true;
}
#endif

#if IMU_HEADING_HOLD_V2_ENABLE
static void UpdateCascadeController(ImuHeadingHoldState_t *state,
                                    float yaw_rad,
                                    float gyro_z_rad_s,
                                    float left_command_rad_s,
                                    float right_command_rad_s,
                                    float wheel_target_limit_rad_s,
                                    float period_s,
                                    ImuHeadingHoldOutput_t *output)
{
    const float left_base_rad_s =
        ClampFloat(left_command_rad_s,
                   -wheel_target_limit_rad_s,
                   wheel_target_limit_rad_s);
    const float right_base_rad_s =
        ClampFloat(right_command_rad_s,
                   -wheel_target_limit_rad_s,
                   wheel_target_limit_rad_s);

    float error_rad =
        WrapAngleRad(state->reference_yaw_rad - yaw_rad);

    if (AbsFloat(error_rad) < IMU_HEADING_HOLD_ERROR_DEADBAND_RAD)
    {
        error_rad = 0.0f;
    }

    const float requested_yaw_rate_rad_s =
        ClampFloat(IMU_HEADING_HOLD_ANGLE_KP * error_rad,
                   -IMU_HEADING_HOLD_YAW_RATE_LIMIT_RAD_S,
                   IMU_HEADING_HOLD_YAW_RATE_LIMIT_RAD_S);

    const float maximum_yaw_rate_change_rad_s =
        IMU_HEADING_HOLD_YAW_RATE_SLEW_RAD_S2 * period_s;

    state->desired_yaw_rate_rad_s +=
        ClampFloat(requested_yaw_rate_rad_s -
                       state->desired_yaw_rate_rad_s,
                   -maximum_yaw_rate_change_rad_s,
                   maximum_yaw_rate_change_rad_s);

    const float yaw_rate_error_rad_s =
        state->desired_yaw_rate_rad_s - gyro_z_rad_s;
    const float rate_ff_correction_rad_s =
        IMU_HEADING_HOLD_KINEMATIC_FF *
        state->desired_yaw_rate_rad_s;
    const float rate_p_correction_rad_s =
        IMU_HEADING_HOLD_RATE_KP * yaw_rate_error_rad_s;

    const float correction_min_rad_s =
        MaxFloat(-IMU_HEADING_HOLD_CORRECTION_LIMIT_RAD_S,
                 MaxFloat(left_base_rad_s - wheel_target_limit_rad_s,
                          -wheel_target_limit_rad_s -
                              right_base_rad_s));
    const float correction_max_rad_s =
        MinFloat(IMU_HEADING_HOLD_CORRECTION_LIMIT_RAD_S,
                 MinFloat(left_base_rad_s + wheel_target_limit_rad_s,
                          wheel_target_limit_rad_s -
                              right_base_rad_s));

    const float candidate_integral_rad_s =
        ClampFloat(
            state->yaw_rate_integral_correction_rad_s +
                (IMU_HEADING_HOLD_RATE_KI *
                 yaw_rate_error_rad_s * period_s),
            -IMU_HEADING_HOLD_RATE_I_LIMIT_RAD_S,
            IMU_HEADING_HOLD_RATE_I_LIMIT_RAD_S);

    const float candidate_raw_correction_rad_s =
        rate_ff_correction_rad_s +
        rate_p_correction_rad_s +
        candidate_integral_rad_s;

    const bool candidate_high_saturated =
        candidate_raw_correction_rad_s > correction_max_rad_s;
    const bool candidate_low_saturated =
        candidate_raw_correction_rad_s < correction_min_rad_s;

    if ((!candidate_high_saturated && !candidate_low_saturated) ||
        (candidate_high_saturated &&
         (yaw_rate_error_rad_s < 0.0f)) ||
        (candidate_low_saturated &&
         (yaw_rate_error_rad_s > 0.0f)))
    {
        state->yaw_rate_integral_correction_rad_s =
            candidate_integral_rad_s;
    }

    const float raw_correction_rad_s =
        rate_ff_correction_rad_s +
        rate_p_correction_rad_s +
        state->yaw_rate_integral_correction_rad_s;
    const float requested_correction_rad_s =
        ClampFloat(raw_correction_rad_s,
                   correction_min_rad_s,
                   correction_max_rad_s);

    const float maximum_correction_change_rad_s =
        IMU_HEADING_HOLD_CORRECTION_SLEW_RAD_S2 * period_s;

    state->correction_rad_s +=
        ClampFloat(requested_correction_rad_s -
                       state->correction_rad_s,
                   -maximum_correction_change_rad_s,
                   maximum_correction_change_rad_s);
    state->correction_rad_s =
        ClampFloat(state->correction_rad_s,
                   correction_min_rad_s,
                   correction_max_rad_s);

    output->reference_yaw_rad = state->reference_yaw_rad;
    output->error_rad = error_rad;
    output->desired_yaw_rate_rad_s =
        state->desired_yaw_rate_rad_s;
    output->yaw_rate_error_rad_s = yaw_rate_error_rad_s;
    output->rate_ff_correction_rad_s = rate_ff_correction_rad_s;
    output->rate_p_correction_rad_s = rate_p_correction_rad_s;
    output->rate_i_correction_rad_s =
        state->yaw_rate_integral_correction_rad_s;
    output->correction_rad_s = state->correction_rad_s;
    output->left_target_rad_s =
        left_base_rad_s - state->correction_rad_s;
    output->right_target_rad_s =
        right_base_rad_s + state->correction_rad_s;
    output->controller_version = 2U;
    output->correction_saturated =
        raw_correction_rad_s != requested_correction_rad_s;
    output->active = true;
}
#endif

void ImuHeadingHold_Update(ImuHeadingHoldState_t *state,
                           bool imu_ready,
                           float yaw_rad,
                           float gyro_z_rad_s,
                           float left_command_rad_s,
                           float right_command_rad_s,
                           float wheel_target_limit_rad_s,
                           float period_s,
                           ImuHeadingHoldOutput_t *output)
{
    if ((state == NULL) || (output == NULL))
    {
        return;
    }

    if (!((period_s > 0.0f) && (period_s <= 1.0f)) ||
        !((wheel_target_limit_rad_s > 0.0f) &&
          (wheel_target_limit_rad_s <= 100.0f)) ||
        !imu_ready ||
        !ImuMeasurementIsValid(yaw_rad, gyro_z_rad_s) ||
        !IsStraightCommand(left_command_rad_s,
                           right_command_rad_s))
    {
        ImuHeadingHold_Reset(state);
        FillPassThrough(left_command_rad_s,
                        right_command_rad_s,
                        output);
        return;
    }

    if (!state->active)
    {
        state->reference_yaw_rad = yaw_rad;
        state->desired_yaw_rate_rad_s = 0.0f;
        state->yaw_rate_integral_correction_rad_s = 0.0f;
        state->correction_rad_s = 0.0f;
        state->active = true;
    }

#if IMU_HEADING_HOLD_V2_ENABLE
    UpdateCascadeController(state,
                            yaw_rad,
                            gyro_z_rad_s,
                            left_command_rad_s,
                            right_command_rad_s,
                            wheel_target_limit_rad_s,
                            period_s,
                            output);
#else
    UpdateLegacyController(state,
                           yaw_rad,
                           gyro_z_rad_s,
                           left_command_rad_s,
                           right_command_rad_s,
                           wheel_target_limit_rad_s,
                           period_s,
                           output);
#endif
}
