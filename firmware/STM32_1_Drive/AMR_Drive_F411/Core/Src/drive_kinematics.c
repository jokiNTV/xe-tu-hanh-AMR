#include "drive_kinematics.h"

#include <stddef.h>

#define DRIVE_PI_F                  (3.14159265358979323846f)
#define DRIVE_FILTER_ZERO_EPS_M_S   (0.0005f)
#define DRIVE_FILTER_MAX_DT_MS      (100U)

typedef struct
{
    uint32_t previous_timestamp_ms;
    float left_filtered_speed_m_s;
    float right_filtered_speed_m_s;
    bool initialized;
} DriveKinematicsState_t;

static DriveKinematicsState_t kinematics_state;

static float wheel_circumference_m(void)
{
    return DRIVE_PI_F * DRIVE_WHEEL_DIAMETER_M;
}

static float clamp_small_speed_to_zero(float speed_m_s)
{
    if ((speed_m_s > -DRIVE_FILTER_ZERO_EPS_M_S) &&
        (speed_m_s < DRIVE_FILTER_ZERO_EPS_M_S))
    {
        return 0.0f;
    }

    return speed_m_s;
}

void DriveKinematics_Init(void)
{
    DriveKinematics_Reset();
}

void DriveKinematics_Reset(void)
{
    kinematics_state.previous_timestamp_ms = 0U;
    kinematics_state.left_filtered_speed_m_s = 0.0f;
    kinematics_state.right_filtered_speed_m_s = 0.0f;
    kinematics_state.initialized = false;
}

bool DriveKinematics_Update(const DriveEncoderSample_t *encoder,
                            DriveKinematicsSample_t *kinematics)
{
    const float circumference_m = wheel_circumference_m();
    const float left_m_per_count =
        circumference_m / DRIVE_ENCODER_LEFT_CPR;
    const float right_m_per_count =
        circumference_m / DRIVE_ENCODER_RIGHT_CPR;
    const float filter_tau_s =
        1.0f / (2.0f * DRIVE_PI_F * DRIVE_SPEED_FILTER_CUTOFF_HZ);
    uint32_t delta_time_ms;
    float delta_time_s;
    float inverse_delta_time_s;
    float filter_alpha;

    if ((encoder == NULL) || (kinematics == NULL))
    {
        return false;
    }

    kinematics->left_position_m =
        (float)encoder->left_total_count * left_m_per_count;
    kinematics->right_position_m =
        (float)encoder->right_total_count * right_m_per_count;
    kinematics->timestamp_ms = encoder->timestamp_ms;

    if (!kinematics_state.initialized)
    {
        kinematics_state.previous_timestamp_ms = encoder->timestamp_ms;
        kinematics_state.initialized = true;

        kinematics->left_speed_raw_m_s = 0.0f;
        kinematics->right_speed_raw_m_s = 0.0f;
        kinematics->left_speed_m_s = 0.0f;
        kinematics->right_speed_m_s = 0.0f;
        kinematics->left_speed_rad_s = 0.0f;
        kinematics->right_speed_rad_s = 0.0f;
        kinematics->left_speed_rpm = 0.0f;
        kinematics->right_speed_rpm = 0.0f;
        return true;
    }

    delta_time_ms =
        encoder->timestamp_ms - kinematics_state.previous_timestamp_ms;

    if (delta_time_ms == 0U)
    {
        return false;
    }

    kinematics_state.previous_timestamp_ms = encoder->timestamp_ms;
    delta_time_s = (float)delta_time_ms * 0.001f;
    inverse_delta_time_s = 1.0f / delta_time_s;

    kinematics->left_speed_raw_m_s =
        (float)encoder->left_delta_count *
        left_m_per_count *
        inverse_delta_time_s;

    kinematics->right_speed_raw_m_s =
        (float)encoder->right_delta_count *
        right_m_per_count *
        inverse_delta_time_s;

    if (delta_time_ms > DRIVE_FILTER_MAX_DT_MS)
    {
        /*
         * Debugger tam dung hoac task bi tre qua lau:
         * khoi dong lai bo loc tu gia tri hien tai de tranh buoc nhay cu.
         */
        kinematics_state.left_filtered_speed_m_s =
            kinematics->left_speed_raw_m_s;
        kinematics_state.right_filtered_speed_m_s =
            kinematics->right_speed_raw_m_s;
    }
    else
    {
        filter_alpha = delta_time_s / (filter_tau_s + delta_time_s);

        kinematics_state.left_filtered_speed_m_s +=
            filter_alpha *
            (kinematics->left_speed_raw_m_s -
             kinematics_state.left_filtered_speed_m_s);

        kinematics_state.right_filtered_speed_m_s +=
            filter_alpha *
            (kinematics->right_speed_raw_m_s -
             kinematics_state.right_filtered_speed_m_s);
    }

    kinematics_state.left_filtered_speed_m_s =
        clamp_small_speed_to_zero(
            kinematics_state.left_filtered_speed_m_s);

    kinematics_state.right_filtered_speed_m_s =
        clamp_small_speed_to_zero(
            kinematics_state.right_filtered_speed_m_s);

    kinematics->left_speed_m_s =
        kinematics_state.left_filtered_speed_m_s;
    kinematics->right_speed_m_s =
        kinematics_state.right_filtered_speed_m_s;

    kinematics->left_speed_rad_s =
        kinematics->left_speed_m_s /
        (DRIVE_WHEEL_DIAMETER_M * 0.5f);

    kinematics->right_speed_rad_s =
        kinematics->right_speed_m_s /
        (DRIVE_WHEEL_DIAMETER_M * 0.5f);

    kinematics->left_speed_rpm =
        (kinematics->left_speed_m_s / circumference_m) * 60.0f;

    kinematics->right_speed_rpm =
        (kinematics->right_speed_m_s / circumference_m) * 60.0f;

    return true;
}
