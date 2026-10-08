#include "imu_heading.h"

#include <math.h>
#include <stddef.h>

#define IMU_PI_RAD                         3.14159265358979323846f
#define IMU_DEG_TO_RAD                     (IMU_PI_RAD / 180.0f)

/* Bring-up / calibration gates. */
#define IMU_CAL_GYRO_LIMIT_DPS             3.0f
#define IMU_CAL_ACCEL_MIN_M_S2             (0.90f * BNO055_GRAVITY_M_S2)
#define IMU_CAL_ACCEL_MAX_M_S2             (1.10f * BNO055_GRAVITY_M_S2)

/*
 * Anti-noise chain, executed at the source before data reaches control/ROS:
 *   BNO055 internal fusion/offset compensation
 *        -> median-of-3 spike reject
 *        -> adaptive zero-rate bias tracker
 *        -> deadband
 *        -> first-order LPF
 *        -> yaw prediction
 *        -> quaternion innovation gate + gentle correction
 */
#define IMU_GYRO_Z_LPF_ALPHA               0.25f
#define IMU_GYRO_Z_DEADBAND_DPS            0.08f

/* Bias is only adapted when the robot looks rotationally stationary. */
#define IMU_BIAS_TRACK_RATE_LIMIT_DPS       0.35f
#define IMU_BIAS_TRACK_ACCEL_MIN_M_S2       (0.97f * BNO055_GRAVITY_M_S2)
#define IMU_BIAS_TRACK_ACCEL_MAX_M_S2       (1.03f * BNO055_GRAVITY_M_S2)
#define IMU_BIAS_TRACK_MIN_SAMPLES          20U
#define IMU_BIAS_TRACK_ALPHA                0.002f

/* Reject an abrupt BNO orientation jump rather than forwarding it downstream. */
#define IMU_YAW_INNOVATION_GATE_RAD         (5.0f * IMU_DEG_TO_RAD)
#define IMU_YAW_CORRECTION_GAIN_MOVING      0.02f
#define IMU_YAW_CORRECTION_GAIN_STATIONARY  0.08f

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float WrapAngleRad(float angle_rad)
{
    while (angle_rad > IMU_PI_RAD)
    {
        angle_rad -= 2.0f * IMU_PI_RAD;
    }

    while (angle_rad < -IMU_PI_RAD)
    {
        angle_rad += 2.0f * IMU_PI_RAD;
    }

    return angle_rad;
}

static float Median3(float a, float b, float c)
{
    if (a > b)
    {
        const float t = a;
        a = b;
        b = t;
    }

    if (b > c)
    {
        const float t = b;
        b = c;
        c = t;
    }

    if (a > b)
    {
        const float t = a;
        a = b;
        b = t;
    }

    return b;
}

static float AccelMagnitudeSquared(const Bno055Sample_t *sample)
{
    return
        (sample->accel_m_s2[0] * sample->accel_m_s2[0]) +
        (sample->accel_m_s2[1] * sample->accel_m_s2[1]) +
        (sample->accel_m_s2[2] * sample->accel_m_s2[2]);
}

static bool InitialCalibrationSampleIsValid(const Bno055Sample_t *sample)
{
    const float accel_squared = AccelMagnitudeSquared(sample);
    const float accel_min_squared =
        IMU_CAL_ACCEL_MIN_M_S2 * IMU_CAL_ACCEL_MIN_M_S2;
    const float accel_max_squared =
        IMU_CAL_ACCEL_MAX_M_S2 * IMU_CAL_ACCEL_MAX_M_S2;

    const bool self_test_ok =
        (sample->self_test_result & 0x0DU) == 0x0DU; /* MCU + GYR + ACC */

    return
        sample->quaternion_valid &&
        (sample->system_status == 5U) && /* sensor fusion running */
        self_test_ok &&
        (sample->calibration_gyro == 3U) &&
        (AbsFloat(sample->gyro_dps[0]) <= IMU_CAL_GYRO_LIMIT_DPS) &&
        (AbsFloat(sample->gyro_dps[1]) <= IMU_CAL_GYRO_LIMIT_DPS) &&
        (AbsFloat(sample->gyro_dps[2]) <= IMU_CAL_GYRO_LIMIT_DPS) &&
        (accel_squared >= accel_min_squared) &&
        (accel_squared <= accel_max_squared);
}

static bool ZeroRateCondition(const Bno055Sample_t *sample,
                              float corrected_gyro_z_dps)
{
    const float accel_squared = AccelMagnitudeSquared(sample);
    const float accel_min_squared =
        IMU_BIAS_TRACK_ACCEL_MIN_M_S2 *
        IMU_BIAS_TRACK_ACCEL_MIN_M_S2;
    const float accel_max_squared =
        IMU_BIAS_TRACK_ACCEL_MAX_M_S2 *
        IMU_BIAS_TRACK_ACCEL_MAX_M_S2;

    return
        (AbsFloat(corrected_gyro_z_dps) <=
         IMU_BIAS_TRACK_RATE_LIMIT_DPS) &&
        (accel_squared >= accel_min_squared) &&
        (accel_squared <= accel_max_squared);
}

static void FillOutput(const ImuHeadingState_t *state,
                       ImuHeadingOutput_t *output)
{
    output->calibration_count = state->calibration_count;
    output->calibration_reset_count =
        state->calibration_reset_count;
    output->gyro_bias_z_dps = state->gyro_bias_z_dps;
    output->gyro_z_dps = state->filtered_gyro_z_dps;
    output->gyro_z_rad_s =
        state->filtered_gyro_z_dps * IMU_DEG_TO_RAD;
    output->yaw_rad = state->yaw_rad;
    output->ready = state->ready;
}

void ImuHeading_Init(ImuHeadingState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    *state = (ImuHeadingState_t){0};
}

void ImuHeading_Update(ImuHeadingState_t *state,
                       const Bno055Sample_t *sample,
                       float period_s,
                       ImuHeadingOutput_t *output)
{
    if ((state == NULL) ||
        (sample == NULL) ||
        (output == NULL) ||
        (period_s <= 0.0f) ||
        (period_s > 0.100f))
    {
        return;
    }

    /* If the BNO fusion stops, fail closed instead of sending a jump into
     * DriveOdometry / TurnController / ROS EKF.
     */
    if ((sample->system_status != 5U) ||
        !sample->quaternion_valid)
    {
        if (state->ready || (state->calibration_count != 0U))
        {
            ++state->calibration_reset_count;
        }

        state->ready = false;
        state->calibration_count = 0U;
        state->gyro_sum_z_dps = 0.0f;
        state->gyro_history_count = 0U;
        state->filtered_gyro_z_dps = 0.0f;
        state->stationary_count = 0U;
        state->sensor_zero_valid = false;
        FillOutput(state, output);
        return;
    }

    if (!state->ready)
    {
        if (!InitialCalibrationSampleIsValid(sample))
        {
            if (state->calibration_count != 0U)
            {
                ++state->calibration_reset_count;
            }

            state->calibration_count = 0U;
            state->gyro_sum_z_dps = 0.0f;
            FillOutput(state, output);
            return;
        }

        state->gyro_sum_z_dps += sample->gyro_dps[2];
        ++state->calibration_count;

        if (state->calibration_count >=
            IMU_HEADING_CALIBRATION_SAMPLES)
        {
            state->gyro_bias_z_dps =
                state->gyro_sum_z_dps /
                (float)state->calibration_count;

            state->gyro_history_dps[0] = sample->gyro_dps[2];
            state->gyro_history_dps[1] = sample->gyro_dps[2];
            state->gyro_history_dps[2] = sample->gyro_dps[2];
            state->gyro_history_count = 3U;
            state->gyro_history_index = 0U;

            state->filtered_gyro_z_dps = 0.0f;
            state->yaw_rad = 0.0f;
            state->sensor_zero_yaw_rad = sample->fused_yaw_rad;
            state->sensor_zero_valid = true;
            state->stationary_count = 0U;
            state->ready = true;
        }

        FillOutput(state, output);
        return;
    }

    /* Layer 1: median-of-3 rejects one-sample electrical / vibration spikes. */
    state->gyro_history_dps[state->gyro_history_index] =
        sample->gyro_dps[2];
    state->gyro_history_index =
        (uint8_t)((state->gyro_history_index + 1U) % 3U);

    if (state->gyro_history_count < 3U)
    {
        ++state->gyro_history_count;
    }

    float gyro_z_dps = sample->gyro_dps[2];
    if (state->gyro_history_count >= 3U)
    {
        gyro_z_dps = Median3(state->gyro_history_dps[0],
                             state->gyro_history_dps[1],
                             state->gyro_history_dps[2]);
    }

    float corrected_gyro_z_dps =
        gyro_z_dps - state->gyro_bias_z_dps;

    /* Layer 2: zero-rate detection + very slow bias tracking. */
    if (ZeroRateCondition(sample, corrected_gyro_z_dps))
    {
        if (state->stationary_count < UINT32_MAX)
        {
            ++state->stationary_count;
        }

        if (state->stationary_count >= IMU_BIAS_TRACK_MIN_SAMPLES)
        {
            state->gyro_bias_z_dps +=
                IMU_BIAS_TRACK_ALPHA *
                (gyro_z_dps - state->gyro_bias_z_dps);

            corrected_gyro_z_dps =
                gyro_z_dps - state->gyro_bias_z_dps;
        }
    }
    else
    {
        state->stationary_count = 0U;
    }

    /* Layer 3: deadband removes residual zero-rate chatter. */
    if (AbsFloat(corrected_gyro_z_dps) <
        IMU_GYRO_Z_DEADBAND_DPS)
    {
        corrected_gyro_z_dps = 0.0f;
    }

    /* Layer 4: first-order low-pass filter. */
    state->filtered_gyro_z_dps +=
        IMU_GYRO_Z_LPF_ALPHA *
        (corrected_gyro_z_dps - state->filtered_gyro_z_dps);

    /* Layer 5: yaw prediction from the filtered rate keeps behavior compatible
     * with the old MPU6050 heading pipeline.
     */
    float predicted_yaw_rad =
        WrapAngleRad(state->yaw_rad +
                     (state->filtered_gyro_z_dps *
                      IMU_DEG_TO_RAD * period_s));

    /* Layer 6: use BNO's own relative quaternion as a sanity reference only.
     * Small innovations are blended gently; abrupt orientation jumps are
     * rejected so they cannot propagate into TURN/ODOM/EKF.
     */
    if (state->sensor_zero_valid)
    {
        const float sensor_relative_yaw_rad =
            WrapAngleRad(sample->fused_yaw_rad -
                         state->sensor_zero_yaw_rad);
        const float innovation_rad =
            WrapAngleRad(sensor_relative_yaw_rad -
                         predicted_yaw_rad);

        if (AbsFloat(innovation_rad) <=
            IMU_YAW_INNOVATION_GATE_RAD)
        {
            const float gain =
                (state->stationary_count >=
                 IMU_BIAS_TRACK_MIN_SAMPLES) ?
                IMU_YAW_CORRECTION_GAIN_STATIONARY :
                IMU_YAW_CORRECTION_GAIN_MOVING;

            predicted_yaw_rad =
                WrapAngleRad(predicted_yaw_rad +
                             (gain * innovation_rad));
        }
        else
        {
            ++state->rejected_orientation_count;
        }
    }

    state->yaw_rad = predicted_yaw_rad;

    FillOutput(state, output);
}

bool ImuHeading_ZeroYaw(ImuHeadingState_t *state,
                        const Bno055Sample_t *sample,
                        ImuHeadingOutput_t *output)
{
    if ((state == NULL) ||
        (sample == NULL) ||
        (output == NULL))
    {
        return false;
    }

    /* Do not allow a zero command to hide an unhealthy IMU. */
    if ((!state->ready) ||
        (!sample->quaternion_valid) ||
        (sample->system_status != 5U))
    {
        FillOutput(state, output);
        return false;
    }

    /* Make the robot's CURRENT physical heading the new relative zero.
     *
     * Preserve:
     *   gyro_bias_z_dps
     *   calibration counters/state
     *   median filter history
     *   stationary detector
     *   READY state
     *
     * Re-anchor both estimators to the same origin:
     *   integrated yaw          -> 0
     *   BNO quaternion relative -> 0
     */
    state->yaw_rad = 0.0f;
    state->sensor_zero_yaw_rad = sample->fused_yaw_rad;
    state->sensor_zero_valid = true;

    /* If the robot is genuinely stationary, removing the tiny residual LPF
     * state makes the displayed zero settle immediately without touching
     * the learned gyro bias.
     */
    if (state->stationary_count >= IMU_BIAS_TRACK_MIN_SAMPLES)
    {
        state->filtered_gyro_z_dps = 0.0f;
    }

    FillOutput(state, output);
    return true;
}
