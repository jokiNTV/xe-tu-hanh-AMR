#include "drive_speed_control.h"

#include <stdbool.h>
#include <stddef.h>

/*
 * Tham so commissioning bao thu. Don vi dau ra la count PWM (0..999).
 * Cac gia tri nay chua phai bo PID cuoi cung; se tune tu log thuc nghiem.
 */
#define DRIVE_CONTROL_KP                        (85.0f)
#define DRIVE_CONTROL_KI                        (30.0f)
#define DRIVE_CONTROL_KFF                       (15.0f)
#define DRIVE_CONTROL_STATIC_PWM                (120.0f)
#define DRIVE_CONTROL_INTEGRAL_LIMIT_PWM        (70.0f)
#define DRIVE_CONTROL_PWM_SLEW_PER_UPDATE       (4.0f)
#define DRIVE_CONTROL_STOP_EPS_RAD_S            (0.05f)
#define DRIVE_CONTROL_ZERO_BRAKE_ENABLE       (1)
#define DRIVE_CONTROL_ZERO_BRAKE_KP           (120.0f)
#define DRIVE_CONTROL_ZERO_BRAKE_PWM_LIMIT    (80.0f)
#define DRIVE_CONTROL_ZERO_BRAKE_SETTLE_RAD_S (0.03f)
/*
 * Feedforward V2 tach rieng tung motor va tung chieu, suy ra tu 6 log
 * chay tren san voi |target| = 2 rad/s. He so slope giu bao thu nhu V1;
 * cac static term la diem khoi dau de giam tai cho integrator.
 */
#define DRIVE_CONTROL_LEFT_FORWARD_STATIC_PWM   (140.0f)
#define DRIVE_CONTROL_RIGHT_FORWARD_STATIC_PWM  (115.0f)
#define DRIVE_CONTROL_LEFT_REVERSE_STATIC_PWM   (115.0f)
#define DRIVE_CONTROL_RIGHT_REVERSE_STATIC_PWM  (115.0f)

/*
 * TEST V5 - low/mid-speed forward feedforward shaping.
 *
 * Current physical evidence:
 * - V2 at target ~= 0.50 rad/s oversped to ~0.82..0.87 rad/s.
 * - V3 removed that overspeed but undersped to ~0.41..0.43 rad/s.
 * - V4 fixed 0.50 rad/s well: mean wheel speed ~= 98.6% of target.
 * - V4 at target 1.00 rad/s still oversped after >4 s:
 *     late measured L/R ~= 1.266 / 1.253 rad/s,
 *     while L/R balance and gyro were already close to straight.
 *
 * Root design issue in V4:
 * - it restored the large V2 forward static terms (140/115 PWM)
 *   by 0.90 rad/s, producing a large feedforward step near 1 rad/s.
 * - PI then had to build a large negative integral for several seconds.
 *
 * V5 keeps the V4 calibration that worked at 0.50 rad/s (67/51 PWM),
 * holds it through 1.20 rad/s, then blends gradually back to the original
 * V2 forward static terms, reaching them at 2.00 rad/s.
 *
 * TEST V5 preserves KP/KI/KFF, reverse behavior, limits, slew, stop logic,
 * zero-brake logic, protocol, odometry and public API.
 */
#define DRIVE_CONTROL_V5_LOW_SPEED_FF_ENABLE              (1)
#define DRIVE_CONTROL_V5_LOW_STATIC_RAMP_END_RAD_S        (0.50f)
#define DRIVE_CONTROL_V5_LOW_STATIC_HOLD_END_RAD_S        (1.20f)
#define DRIVE_CONTROL_V5_FULL_STATIC_RAD_S                (2.00f)
#define DRIVE_CONTROL_V5_LEFT_FORWARD_LOW_STATIC_PWM      (67.0f)
#define DRIVE_CONTROL_V5_RIGHT_FORWARD_LOW_STATIC_PWM     (51.0f)

typedef struct
{
    float integral_pwm;
    float previous_output_pwm;
    float previous_target_rad_s;
} DriveAxisControlState_t;

typedef struct
{
    DriveAxisControlState_t left;
    DriveAxisControlState_t right;
} DriveSpeedControlState_t;

static DriveSpeedControlState_t control_state;

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ClampFloat(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

static int16_t FloatToInt16Rounded(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}

static void ResetAxis(DriveAxisControlState_t *axis)
{
    axis->integral_pwm = 0.0f;
    axis->previous_output_pwm = 0.0f;
    axis->previous_target_rad_s = 0.0f;
}

static float LimitOutputToTargetDirection(float output_pwm,
                                          float target_rad_s)
{
    /* Commissioning chi cho phep coast khi qua toc, khong phanh dao chieu. */
    if ((target_rad_s > 0.0f) && (output_pwm < 0.0f))
    {
        return 0.0f;
    }

    if ((target_rad_s < 0.0f) && (output_pwm > 0.0f))
    {
        return 0.0f;
    }

    return output_pwm;
}

static float ApplySlewLimit(float requested_pwm, float previous_pwm)
{
    float delta = requested_pwm - previous_pwm;

    delta = ClampFloat(delta,
                       -DRIVE_CONTROL_PWM_SLEW_PER_UPDATE,
                       DRIVE_CONTROL_PWM_SLEW_PER_UPDATE);

    return previous_pwm + delta;
}

#if !DRIVE_SPEED_CONTROLLER_V2_ENABLE
static int16_t UpdateAxisLegacy(DriveAxisControlState_t *axis,
                                float target_rad_s,
                                float measured_rad_s,
                                float *limited_target_rad_s,
                                float *error_rad_s)
{
    float target;
    float error;
    float feedforward_pwm;
    float candidate_pwm;
    float requested_pwm;
    float output_pwm;
    bool target_reversed;
    bool allow_integrator;

    target = ClampFloat(target_rad_s,
                        -DRIVE_CONTROL_TARGET_LIMIT_RAD_S,
                        DRIVE_CONTROL_TARGET_LIMIT_RAD_S);

    *limited_target_rad_s = target;

    if (AbsFloat(target) < DRIVE_CONTROL_STOP_EPS_RAD_S)
    {
        ResetAxis(axis);
        *error_rad_s = 0.0f;
        return 0;
    }

    target_reversed =
        ((axis->previous_target_rad_s > 0.0f) && (target < 0.0f)) ||
        ((axis->previous_target_rad_s < 0.0f) && (target > 0.0f));

    if (target_reversed)
    {
        /* Tao it nhat mot chu ky PWM = 0 truoc khi doi chieu. */
        ResetAxis(axis);
        axis->previous_target_rad_s = target;
        *error_rad_s = target - measured_rad_s;
        return 0;
    }

    error = target - measured_rad_s;
    *error_rad_s = error;

    feedforward_pwm = DRIVE_CONTROL_KFF * target;

    if (target > 0.0f)
    {
        feedforward_pwm += DRIVE_CONTROL_STATIC_PWM;
    }
    else
    {
        feedforward_pwm -= DRIVE_CONTROL_STATIC_PWM;
    }

    candidate_pwm = feedforward_pwm +
                    (DRIVE_CONTROL_KP * error) +
                    axis->integral_pwm;

    allow_integrator =
        ((candidate_pwm < (float)DRIVE_CONTROL_PWM_LIMIT) &&
         (candidate_pwm > -(float)DRIVE_CONTROL_PWM_LIMIT)) ||
        ((candidate_pwm >= (float)DRIVE_CONTROL_PWM_LIMIT) &&
         (error < 0.0f)) ||
        ((candidate_pwm <= -(float)DRIVE_CONTROL_PWM_LIMIT) &&
         (error > 0.0f));

    if (allow_integrator)
    {
        axis->integral_pwm +=
            DRIVE_CONTROL_KI * error * DRIVE_CONTROL_PERIOD_S;

        axis->integral_pwm =
            ClampFloat(axis->integral_pwm,
                       -DRIVE_CONTROL_INTEGRAL_LIMIT_PWM,
                       DRIVE_CONTROL_INTEGRAL_LIMIT_PWM);
    }

    requested_pwm = feedforward_pwm +
                    (DRIVE_CONTROL_KP * error) +
                    axis->integral_pwm;

    requested_pwm =
        LimitOutputToTargetDirection(requested_pwm, target);

    requested_pwm =
        ClampFloat(requested_pwm,
                   -(float)DRIVE_CONTROL_PWM_LIMIT,
                   (float)DRIVE_CONTROL_PWM_LIMIT);

    output_pwm =
        ApplySlewLimit(requested_pwm, axis->previous_output_pwm);

    axis->previous_output_pwm = output_pwm;
    axis->previous_target_rad_s = target;

    return FloatToInt16Rounded(output_pwm);
}
#endif

#if DRIVE_SPEED_CONTROLLER_V2_ENABLE
#if DRIVE_CONTROL_V5_LOW_SPEED_FF_ENABLE
static float ComputeV5ForwardStaticPwm(float target_rad_s,
                                       float low_static_pwm,
                                       float full_static_pwm)
{
    const float speed_rad_s = AbsFloat(target_rad_s);

    /*
     * Zone A: 0 -> 0.50 rad/s.
     * Ramp static PWM smoothly from zero to the physically validated
     * low-speed values (67 left / 51 right).
     */
    if (speed_rad_s < DRIVE_CONTROL_V5_LOW_STATIC_RAMP_END_RAD_S)
    {
        const float scale =
            speed_rad_s / DRIVE_CONTROL_V5_LOW_STATIC_RAMP_END_RAD_S;
        return low_static_pwm * scale;
    }

    /*
     * Zone B: 0.50 -> 1.20 rad/s.
     * Hold the validated low static term. V4 restored 140/115 too early
     * and caused ~25% overspeed around target = 1.00 rad/s.
     */
    if (speed_rad_s <= DRIVE_CONTROL_V5_LOW_STATIC_HOLD_END_RAD_S)
    {
        return low_static_pwm;
    }

    /*
     * Zone C: 1.20 -> 2.00 rad/s.
     * Gradually restore the original V2 static term.
     */
    if (speed_rad_s < DRIVE_CONTROL_V5_FULL_STATIC_RAD_S)
    {
        const float blend =
            (speed_rad_s - DRIVE_CONTROL_V5_LOW_STATIC_HOLD_END_RAD_S) /
            (DRIVE_CONTROL_V5_FULL_STATIC_RAD_S -
             DRIVE_CONTROL_V5_LOW_STATIC_HOLD_END_RAD_S);

        return low_static_pwm +
               ((full_static_pwm - low_static_pwm) * blend);
    }

    return full_static_pwm;
}
#endif

static float ComputeV2Feedforward(float target_rad_s,
                                  float forward_static_pwm,
                                  float forward_low_static_pwm,
                                  float reverse_static_pwm)
{
    float feedforward_pwm = DRIVE_CONTROL_KFF * target_rad_s;

    if (target_rad_s > 0.0f)
    {
#if DRIVE_CONTROL_V5_LOW_SPEED_FF_ENABLE
        feedforward_pwm +=
            ComputeV5ForwardStaticPwm(target_rad_s,
                                      forward_low_static_pwm,
                                      forward_static_pwm);
#else
        feedforward_pwm += forward_static_pwm;
#endif
    }
    else
    {
        /* Reverse feedforward is intentionally unchanged in TEST V5. */
        feedforward_pwm -= reverse_static_pwm;
    }

    return feedforward_pwm;
}

static float BoundRequestedPwm(float requested_pwm,
                               float target_rad_s)
{
    return ClampFloat(
        LimitOutputToTargetDirection(requested_pwm, target_rad_s),
        -(float)DRIVE_CONTROL_PWM_LIMIT,
        (float)DRIVE_CONTROL_PWM_LIMIT);
}

static int16_t UpdateAxisV2(DriveAxisControlState_t *axis,
                            float target_rad_s,
                            float measured_rad_s,
                            float forward_static_pwm,
                            float forward_low_static_pwm,
                            float reverse_static_pwm,
                            float *limited_target_rad_s,
                            float *error_rad_s)
{
    const float target =
        ClampFloat(target_rad_s,
                   -DRIVE_CONTROL_TARGET_LIMIT_RAD_S,
                   DRIVE_CONTROL_TARGET_LIMIT_RAD_S);

    *limited_target_rad_s = target;

    if (AbsFloat(target) < DRIVE_CONTROL_STOP_EPS_RAD_S)
    {
        ResetAxis(axis);
        *error_rad_s = 0.0f;
        return 0;
    }

    const bool target_reversed =
        ((axis->previous_target_rad_s > 0.0f) && (target < 0.0f)) ||
        ((axis->previous_target_rad_s < 0.0f) && (target > 0.0f));

    if (target_reversed)
    {
        ResetAxis(axis);
        axis->previous_target_rad_s = target;
        *error_rad_s = target - measured_rad_s;
        return 0;
    }

    const float error = target - measured_rad_s;
    const float feedforward_pwm =
        ComputeV2Feedforward(target,
                             forward_static_pwm,
                             forward_low_static_pwm,
                             reverse_static_pwm);

    *error_rad_s = error;

    const float candidate_integral_pwm =
        ClampFloat(axis->integral_pwm +
                       (DRIVE_CONTROL_KI * error *
                        DRIVE_CONTROL_PERIOD_S),
                   -DRIVE_CONTROL_INTEGRAL_LIMIT_PWM,
                   DRIVE_CONTROL_INTEGRAL_LIMIT_PWM);

    const float candidate_requested_pwm =
        feedforward_pwm +
        (DRIVE_CONTROL_KP * error) +
        candidate_integral_pwm;
    const float candidate_bounded_pwm =
        BoundRequestedPwm(candidate_requested_pwm, target);
    const float candidate_output_pwm =
        ApplySlewLimit(candidate_bounded_pwm,
                       axis->previous_output_pwm);

    const bool limited_high =
        candidate_output_pwm < candidate_requested_pwm;
    const bool limited_low =
        candidate_output_pwm > candidate_requested_pwm;

    /*
     * Conditional integration tinh ca saturation va slew-rate limit.
     * Khi actuator chua theo kip lenh, khong cho I tich luy them theo
     * cung chieu; van cho I rut khoi saturation.
     */
    if ((!limited_high && !limited_low) ||
        (limited_high && (error < 0.0f)) ||
        (limited_low && (error > 0.0f)))
    {
        axis->integral_pwm = candidate_integral_pwm;
    }

    const float requested_pwm =
        feedforward_pwm +
        (DRIVE_CONTROL_KP * error) +
        axis->integral_pwm;
    const float bounded_pwm =
        BoundRequestedPwm(requested_pwm, target);
    const float output_pwm =
        ApplySlewLimit(bounded_pwm, axis->previous_output_pwm);

    axis->previous_output_pwm = output_pwm;
    axis->previous_target_rad_s = target;

    return FloatToInt16Rounded(output_pwm);
}
#endif

void DriveSpeedControl_Init(void)
{
    DriveSpeedControl_Reset();
}

void DriveSpeedControl_Reset(void)
{
    ResetAxis(&control_state.left);
    ResetAxis(&control_state.right);
}

void DriveSpeedControl_Update(float left_target_rad_s,
                              float right_target_rad_s,
                              float left_measured_rad_s,
                              float right_measured_rad_s,
                              DriveSpeedControlOutput_t *output)
{
    if (output == NULL)
    {
        return;
    }

#if DRIVE_SPEED_CONTROLLER_V2_ENABLE
    output->left_pwm =
        UpdateAxisV2(&control_state.left,
                     left_target_rad_s,
                     left_measured_rad_s,
                     DRIVE_CONTROL_LEFT_FORWARD_STATIC_PWM,
                     DRIVE_CONTROL_V5_LEFT_FORWARD_LOW_STATIC_PWM,
                     DRIVE_CONTROL_LEFT_REVERSE_STATIC_PWM,
                     &output->left_target_rad_s,
                     &output->left_error_rad_s);

    output->right_pwm =
        UpdateAxisV2(&control_state.right,
                     right_target_rad_s,
                     right_measured_rad_s,
                     DRIVE_CONTROL_RIGHT_FORWARD_STATIC_PWM,
                     DRIVE_CONTROL_V5_RIGHT_FORWARD_LOW_STATIC_PWM,
                     DRIVE_CONTROL_RIGHT_REVERSE_STATIC_PWM,
                     &output->right_target_rad_s,
                     &output->right_error_rad_s);
#else
    output->left_pwm =
        UpdateAxisLegacy(&control_state.left,
                         left_target_rad_s,
                         left_measured_rad_s,
                         &output->left_target_rad_s,
                         &output->left_error_rad_s);

    output->right_pwm =
        UpdateAxisLegacy(&control_state.right,
                         right_target_rad_s,
                         right_measured_rad_s,
                         &output->right_target_rad_s,
                         &output->right_error_rad_s);
#endif
}
