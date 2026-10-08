#include "turn_controller.h"

#include <math.h>
#include <stddef.h>

/*
 * V9 bidirectional terminal-yaw lock + measured breakaway fix.
 *
 * Muc tieu:
 * 1) Giu V8 terminal hold, no-oscillation va backward-bias disable.
 * 2) Dung brake decel 0.45 rad/s^2 theo du lieu dung quay tren xe.
 * 3) Cho RUNNING/near-min 0.05 lam breakaway den 1 do cuoi.
 * 4) Trong TERMINAL HOLD, neu than xe dung ma con sai >0.5 do,
 *    dung breakaway boost 0.06 rad/s; khi da chuyen dong thi tra ve PD.
 * 5) DONE chi khi yaw error <=0.5 do va gyro <=1 do/s lien tuc 400 ms.
 *
 * Khong thay doi wheel PI, motor mapping, odometry, USB/telemetry,
 * heading hold hay route.
 */

#define TURN_CONTROLLER_SOFT_APPROACH_RAD (0.06981317008f) /* 4.0 deg */
#define TURN_CONTROLLER_NEAR_MIN_YAW_RATE_RAD_S (0.050f)
#define TURN_CONTROLLER_BACKWARD_BIAS_RATIO (0.225f)
#define TURN_CONTROLLER_BACKWARD_BIAS_MAX_RAD_S (0.045f)
#define TURN_CONTROLLER_EARLY_CAPTURE_MIN_GYRO_RAD_S (0.030f)
/* V7: dung gyro thuc te de uoc luong goc con troi sau khi ZERO.
 * stop_angle = gyro^2 / (2 * BRAKE_DECEL)
 * Chi cho phep early-capture trong SOFT_APPROACH 4 do cuoi.
 */
/* V6:
 * - Bias khong con co dinh -0.045 rad/s o moi toc do.
 * - Bias ty le theo |yaw_rate_command| de giu cung ty le L/R khi giam toc.
 * - Tai yaw = 0.20 rad/s: bias = -0.045 rad/s nhu V4/V5.
 * - Tai yaw = 0.05 rad/s: bias chi con -0.01125 rad/s.
 * - Clamp khong vuot qua -0.045 rad/s.
 */

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

static float CopySignFloat(float magnitude, float sign_source)
{
    return (sign_source < 0.0f) ? -magnitude : magnitude;
}

static float WrapAngleRad(float angle_rad)
{
    while (angle_rad > TURN_CONTROLLER_PI_RAD)
    {
        angle_rad -= 2.0f * TURN_CONTROLLER_PI_RAD;
    }

    while (angle_rad < -TURN_CONTROLLER_PI_RAD)
    {
        angle_rad += 2.0f * TURN_CONTROLLER_PI_RAD;
    }

    return angle_rad;
}

static bool ImuMeasurementIsValid(float yaw_rad, float gyro_z_rad_s)
{
    /* Cac so sanh nay cung loai NaN. */
    return (yaw_rad >= -TURN_CONTROLLER_PI_RAD) &&
           (yaw_rad <= TURN_CONTROLLER_PI_RAD) &&
           (gyro_z_rad_s >= -20.0f) &&
           (gyro_z_rad_s <= 20.0f);
}

static void FillOutput(const TurnControllerState_t *state,
                       float error_rad,
                       TurnControllerOutput_t *output)
{
    const float wheel_scale =
        TURN_CONTROLLER_WHEEL_SEPARATION_M /
        (2.0f * TURN_CONTROLLER_WHEEL_RADIUS_M);

    output->status = state->status;
    output->target_relative_angle_rad =
        state->target_relative_angle_rad;
    output->traveled_angle_rad = state->traveled_angle_rad;
    output->error_rad = error_rad;
    output->yaw_rate_command_rad_s =
        state->yaw_rate_command_rad_s;
    float common_bias_rad_s = 0.0f;

    /*
     * V6: backward common-mode SCALE theo yaw command.
     *
     * Ly do:
     * Neu giu bias co dinh -0.045 rad/s trong luc yaw command giam xuong
     * 0.05 rad/s, bias tro thanh qua lon so voi wheel target chinh.
     * Dieu nay lam mat can doi pivot o vung cuoi va co the tao slip/drift.
     *
     * Bias scale:
     *   bias_mag = ratio * |yaw_rate_command|
     *   ratio = 0.225
     *   max   = 0.045 rad/s
     *
     * O 0.20 rad/s -> 0.045 rad/s
     * O 0.05 rad/s -> 0.01125 rad/s
     */
    if ((state->status == TURN_CONTROLLER_RUNNING) &&
        (AbsFloat(error_rad) > TURN_CONTROLLER_TERMINAL_BIAS_DISABLE_RAD) &&
        (AbsFloat(state->target_relative_angle_rad) > 0.001f) &&
        (AbsFloat(state->yaw_rate_command_rad_s) > 0.001f))
    {
        float bias_magnitude_rad_s =
            TURN_CONTROLLER_BACKWARD_BIAS_RATIO *
            AbsFloat(state->yaw_rate_command_rad_s);

        if (bias_magnitude_rad_s >
            TURN_CONTROLLER_BACKWARD_BIAS_MAX_RAD_S)
        {
            bias_magnitude_rad_s =
                TURN_CONTROLLER_BACKWARD_BIAS_MAX_RAD_S;
        }

        common_bias_rad_s = -bias_magnitude_rad_s;
    }

    output->left_target_rad_s =
        -state->yaw_rate_command_rad_s * wheel_scale +
        common_bias_rad_s;
    output->right_target_rad_s =
        state->yaw_rate_command_rad_s * wheel_scale +
        common_bias_rad_s;
    output->active =
        (state->status == TURN_CONTROLLER_RUNNING) ||
        (state->status == TURN_CONTROLLER_SETTLING);
    output->completed = (state->status == TURN_CONTROLLER_DONE);
    output->fault =
        (state->status == TURN_CONTROLLER_FAULT_TIMEOUT) ||
        (state->status == TURN_CONTROLLER_FAULT_IMU) ||
        (state->status == TURN_CONTROLLER_FAULT_ARGUMENT);
}

static void UpdateSettling(TurnControllerState_t *state,
                           float error_rad,
                           float gyro_z_rad_s,
                           float period_s,
                           uint32_t now_ms)
{
    /*
     * V9: TURN_CONTROLLER_SETTLING duoc dung nhu TERMINAL YAW HOLD.
     *
     * - Khong con latch ZERO roi cho quan tinh tu do.
     * - Cho phep correction hai chieu nho quanh target IMU/ODOM turn.
     * - Backward common-bias bi tat trong SETTLING boi FillOutput().
     * - DONE chi khi yaw error + gyro cung sach lien tuc 400 ms.
     */
    state->status = TURN_CONTROLLER_SETTLING;

    const float error_magnitude_rad = AbsFloat(error_rad);
    const float gyro_magnitude_rad_s = AbsFloat(gyro_z_rad_s);

    if ((error_magnitude_rad <=
         TURN_CONTROLLER_TERMINAL_HOLD_TOLERANCE_RAD) &&
        (gyro_magnitude_rad_s <=
         TURN_CONTROLLER_TERMINAL_HOLD_GYRO_LIMIT_RAD_S))
    {
        state->yaw_rate_command_rad_s = 0.0f;

        if (!state->settling_timer_active)
        {
            state->settling_since_ms = now_ms;
            state->settling_timer_active = true;
        }
        else if ((now_ms - state->settling_since_ms) >=
                 TURN_CONTROLLER_TERMINAL_HOLD_SETTLE_TIME_MS)
        {
            state->status = TURN_CONTROLLER_DONE;
        }
        return;
    }

    state->settling_timer_active = false;

    float requested_yaw_rate_rad_s =
        (TURN_CONTROLLER_TERMINAL_HOLD_KP * error_rad) -
        (TURN_CONTROLLER_TERMINAL_HOLD_KD * gyro_z_rad_s);

    requested_yaw_rate_rad_s =
        ClampFloat(requested_yaw_rate_rad_s,
                   -TURN_CONTROLLER_TERMINAL_HOLD_MAX_YAW_RATE_RAD_S,
                   TURN_CONTROLLER_TERMINAL_HOLD_MAX_YAW_RATE_RAD_S);

    /*
     * V9 small-angle breakaway:
     * - Neu chassis dang gan nhu dung va con ngoai 0.5 deg, cho mot floor
     *   0.06 rad/s de pha ma sat/caster.
     * - Khi gyro da co chuyen dong, bo boost va de PD + floor 0.02 dieu
     *   khien mem. Nhu vay khong giu 0.06 suot pha terminal.
     */
    if ((error_magnitude_rad >
         TURN_CONTROLLER_TERMINAL_HOLD_TOLERANCE_RAD) &&
        (gyro_magnitude_rad_s <=
         TURN_CONTROLLER_TERMINAL_BREAKAWAY_GYRO_RAD_S) &&
        (AbsFloat(requested_yaw_rate_rad_s) <
         TURN_CONTROLLER_TERMINAL_BREAKAWAY_YAW_RATE_RAD_S))
    {
        requested_yaw_rate_rad_s =
            CopySignFloat(TURN_CONTROLLER_TERMINAL_BREAKAWAY_YAW_RATE_RAD_S,
                          error_rad);
    }
    else if ((error_magnitude_rad >
              TURN_CONTROLLER_TERMINAL_HOLD_TOLERANCE_RAD) &&
             (AbsFloat(requested_yaw_rate_rad_s) <
              TURN_CONTROLLER_TERMINAL_HOLD_MIN_YAW_RATE_RAD_S))
    {
        requested_yaw_rate_rad_s =
            CopySignFloat(TURN_CONTROLLER_TERMINAL_HOLD_MIN_YAW_RATE_RAD_S,
                          error_rad);
    }

    /* Neu correction can doi dau, zero truoc de khong tiep tuc day sai huong. */
    if ((state->yaw_rate_command_rad_s * requested_yaw_rate_rad_s) < 0.0f)
    {
        state->yaw_rate_command_rad_s = 0.0f;
    }

    const float maximum_change_rad_s =
        TURN_CONTROLLER_YAW_RATE_SLEW_RAD_S2 * period_s;
    const float command_change_rad_s =
        ClampFloat(requested_yaw_rate_rad_s -
                       state->yaw_rate_command_rad_s,
                   -maximum_change_rad_s,
                   maximum_change_rad_s);

    state->yaw_rate_command_rad_s += command_change_rad_s;
}

void TurnController_Init(TurnControllerState_t *state)
{
    TurnController_Cancel(state);
}

void TurnController_Cancel(TurnControllerState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->status = TURN_CONTROLLER_IDLE;
    state->target_relative_angle_rad = 0.0f;
    state->previous_yaw_rad = 0.0f;
    state->traveled_angle_rad = 0.0f;
    state->yaw_rate_command_rad_s = 0.0f;
    state->max_yaw_rate_rad_s = TURN_CONTROLLER_MAX_YAW_RATE_RAD_S;
    state->started_at_ms = 0U;
    state->timeout_ms = 0U;
    state->settling_since_ms = 0U;
    state->settling_timer_active = false;
}

bool TurnController_Start(TurnControllerState_t *state,
                          float relative_angle_rad,
                          float max_yaw_rate_rad_s,
                          uint32_t timeout_ms,
                          float current_yaw_rad,
                          uint32_t now_ms)
{
    if (state == NULL)
    {
        return false;
    }

    TurnController_Cancel(state);

    const bool argument_valid =
        /* Protocol dung mrad: 180 do duoc lam tron thanh 3.142 rad. */
        (relative_angle_rad >= -(TURN_CONTROLLER_PI_RAD + 0.001f)) &&
        (relative_angle_rad <= (TURN_CONTROLLER_PI_RAD + 0.001f)) &&
        (max_yaw_rate_rad_s >= TURN_CONTROLLER_MIN_YAW_RATE_RAD_S) &&
        (max_yaw_rate_rad_s <= 0.70f) &&
        (timeout_ms >= TURN_CONTROLLER_MIN_TIMEOUT_MS) &&
        (timeout_ms <= TURN_CONTROLLER_MAX_TIMEOUT_MS) &&
        (current_yaw_rad >= -TURN_CONTROLLER_PI_RAD) &&
        (current_yaw_rad <= TURN_CONTROLLER_PI_RAD);

    if (!argument_valid)
    {
        state->status = TURN_CONTROLLER_FAULT_ARGUMENT;
        return false;
    }

    state->status = TURN_CONTROLLER_RUNNING;
    state->target_relative_angle_rad = relative_angle_rad;
    state->previous_yaw_rad = current_yaw_rad;
    state->traveled_angle_rad = 0.0f;
    state->yaw_rate_command_rad_s = 0.0f;
    state->max_yaw_rate_rad_s = max_yaw_rate_rad_s;
    state->started_at_ms = now_ms;
    state->timeout_ms = timeout_ms;
    return true;
}

void TurnController_Update(TurnControllerState_t *state,
                           bool imu_ready,
                           float yaw_rad,
                           float gyro_z_rad_s,
                           float period_s,
                           uint32_t now_ms,
                           TurnControllerOutput_t *output)
{
    if ((state == NULL) || (output == NULL))
    {
        return;
    }

    if ((state->status != TURN_CONTROLLER_RUNNING) &&
        (state->status != TURN_CONTROLLER_SETTLING))
    {
        FillOutput(state,
                   state->target_relative_angle_rad -
                       state->traveled_angle_rad,
                   output);
        return;
    }

    if (!imu_ready ||
        !ImuMeasurementIsValid(yaw_rad, gyro_z_rad_s) ||
        !((period_s > 0.0f) && (period_s <= 0.1f)))
    {
        state->status = TURN_CONTROLLER_FAULT_IMU;
        state->yaw_rate_command_rad_s = 0.0f;
        FillOutput(state, 0.0f, output);
        return;
    }

    if ((now_ms - state->started_at_ms) >= state->timeout_ms)
    {
        state->status = TURN_CONTROLLER_FAULT_TIMEOUT;
        state->yaw_rate_command_rad_s = 0.0f;
        FillOutput(state,
                   state->target_relative_angle_rad -
                       state->traveled_angle_rad,
                   output);
        return;
    }

    const float yaw_delta_rad =
        WrapAngleRad(yaw_rad - state->previous_yaw_rad);
    state->previous_yaw_rad = yaw_rad;
    state->traveled_angle_rad += yaw_delta_rad;

    const float error_rad =
        state->target_relative_angle_rad -
        state->traveled_angle_rad;

    /*
     * Neu da vao SETTLING thi TERMINAL HOLD nam quyen cho toi DONE.
     * Hold co the correction hai chieu nho, nhung khong quay lai RUNNING.
     */
    if (state->status == TURN_CONTROLLER_SETTLING)
    {
        UpdateSettling(state, error_rad, gyro_z_rad_s, period_s, now_ms);
        FillOutput(state, error_rad, output);
        return;
    }

    const float remaining_magnitude_rad = AbsFloat(error_rad);

    /* V9: vao TERMINAL HOLD o 1 do cuoi; 2-4 do dung near-min 0.05. */
    if (remaining_magnitude_rad <= TURN_CONTROLLER_TERMINAL_HOLD_ENTER_RAD)
    {
        state->settling_timer_active = false;
        UpdateSettling(state, error_rad, gyro_z_rad_s, period_s, now_ms);
        FillOutput(state, error_rad, output);
        return;
    }

    /*
     * Capture co ban:
     * - Da vao tolerance, HOAC
     * - Da di qua target.
     *
     * Khi capture: vao TERMINAL HOLD.
     * Hold se sua nho neu quan tinh/caster lam lech target.
     */
    const bool target_crossed =
        ((state->target_relative_angle_rad > 0.0f) &&
         (error_rad <= 0.0f)) ||
        ((state->target_relative_angle_rad < 0.0f) &&
         (error_rad >= 0.0f));

    if ((remaining_magnitude_rad <= TURN_CONTROLLER_TOLERANCE_RAD) ||
        target_crossed)
    {
        state->settling_timer_active = false;
        UpdateSettling(state, error_rad, gyro_z_rad_s, period_s, now_ms);
        FillOutput(state, error_rad, output);
        return;
    }

    /*
     * V7 PREDICTIVE EARLY CAPTURE
     *
     * Van de V6:
     * controller doi toi target/tolerance moi ZERO.
     * Neu than xe con yaw-rate thuc te, quan tinh/caster lam xe troi them
     * sau ZERO. Cac run da co overshoot vat ly thay doi tu ~0.4 do toi >4 do.
     *
     * Giai phap:
     * dung gyro thuc te de uoc luong stopping angle:
     *
     *     stop_angle = gyro^2 / (2 * brake_decel)
     *
     * Neu goc con lai <= stopping angle, vao TERMINAL HOLD som.
     * Nhu vay brake point tu dong thay doi theo toc do thuc te tung run.
     *
     * Chi kich hoat:
     * - trong 4 do cuoi,
     * - gyro dang quay dung huong target,
     * - gyro du lon de phep tinh co y nghia.
     */
    const float gyro_magnitude_rad_s = AbsFloat(gyro_z_rad_s);
    const bool moving_toward_target =
        ((state->target_relative_angle_rad > 0.0f) &&
         (gyro_z_rad_s > 0.0f)) ||
        ((state->target_relative_angle_rad < 0.0f) &&
         (gyro_z_rad_s < 0.0f));

    if (moving_toward_target &&
        (gyro_magnitude_rad_s >=
         TURN_CONTROLLER_EARLY_CAPTURE_MIN_GYRO_RAD_S) &&
        (remaining_magnitude_rad <= TURN_CONTROLLER_SOFT_APPROACH_RAD))
    {
        const float predicted_stop_angle_rad =
            (gyro_magnitude_rad_s * gyro_magnitude_rad_s) /
            (2.0f * TURN_CONTROLLER_BRAKE_DECEL_RAD_S2);

        if (remaining_magnitude_rad <= predicted_stop_angle_rad)
        {
            state->settling_timer_active = false;
            UpdateSettling(state, error_rad, gyro_z_rad_s, period_s, now_ms);
            FillOutput(state, error_rad, output);
            return;
        }
    }

    state->status = TURN_CONTROLLER_RUNNING;
    state->settling_timer_active = false;

    float requested_yaw_rate_rad_s =
        (TURN_CONTROLLER_KP * error_rad) -
        (TURN_CONTROLLER_KD * gyro_z_rad_s);

    /*
     * Gioi han toc do quay theo goc con lai: v^2 = 2*a*s.
     */
    const float braking_rate_limit_rad_s =
        sqrtf(2.0f * TURN_CONTROLLER_BRAKE_DECEL_RAD_S2 *
              remaining_magnitude_rad);
    float permitted_yaw_rate_rad_s =
        (braking_rate_limit_rad_s < state->max_yaw_rate_rad_s) ?
        braking_rate_limit_rad_s : state->max_yaw_rate_rad_s;

    /* V8: 12 do cuoi khong cho quay nhanh hon 0.18 rad/s. */
    if ((remaining_magnitude_rad <= TURN_CONTROLLER_FINAL_APPROACH_RAD) &&
        (permitted_yaw_rate_rad_s >
         TURN_CONTROLLER_FINAL_MAX_YAW_RATE_RAD_S))
    {
        permitted_yaw_rate_rad_s =
            TURN_CONTROLLER_FINAL_MAX_YAW_RATE_RAD_S;
    }

    requested_yaw_rate_rad_s =
        ClampFloat(requested_yaw_rate_rad_s,
                   -permitted_yaw_rate_rad_s,
                   permitted_yaw_rate_rad_s);

    /*
     * Khong cho controller ra lenh quay NGUOC huong command ban dau.
     * KD duoc phep giam ve 0 de ham, nhung khong duoc doi dau va lac lai.
     */
    if ((state->target_relative_angle_rad > 0.0f) &&
        (requested_yaw_rate_rad_s < 0.0f))
    {
        requested_yaw_rate_rad_s = 0.0f;
    }
    else if ((state->target_relative_angle_rad < 0.0f) &&
             (requested_yaw_rate_rad_s > 0.0f))
    {
        requested_yaw_rate_rad_s = 0.0f;
    }

    /*
     * V3 minimum yaw-rate 2 cap:
     *
     * - Ngoai 4 do:
     *     giu MIN_YAW_RATE goc de dam bao luc quay / breakaway.
     *
     * - Trong 4 do nhung CHUA vao tolerance:
     *     giu mot near-min nho 0.07 rad/s.
     *     Muc tieu la khong bi dung non o ~1 do do ma sat tinh.
     *
     * - Khi vao tolerance:
     *     nhanh capture phia tren se ZERO + latch SETTLING.
     *
     * - Neu vuot target:
     *     target_crossed phia tren se ZERO ngay, khong dao chieu sua lai.
     */
    if (remaining_magnitude_rad > TURN_CONTROLLER_SOFT_APPROACH_RAD)
    {
        if (AbsFloat(requested_yaw_rate_rad_s) <
            TURN_CONTROLLER_MIN_YAW_RATE_RAD_S)
        {
            requested_yaw_rate_rad_s =
                CopySignFloat(TURN_CONTROLLER_MIN_YAW_RATE_RAD_S,
                              error_rad);
        }
    }
    else
    {
        if (AbsFloat(requested_yaw_rate_rad_s) <
            TURN_CONTROLLER_NEAR_MIN_YAW_RATE_RAD_S)
        {
            requested_yaw_rate_rad_s =
                CopySignFloat(TURN_CONTROLLER_NEAR_MIN_YAW_RATE_RAD_S,
                              error_rad);
        }
    }

    const float maximum_change_rad_s =
        TURN_CONTROLLER_YAW_RATE_SLEW_RAD_S2 * period_s;
    const float command_change_rad_s =
        ClampFloat(requested_yaw_rate_rad_s -
                       state->yaw_rate_command_rad_s,
                   -maximum_change_rad_s,
                   maximum_change_rad_s);

    state->yaw_rate_command_rad_s += command_change_rad_s;

    /*
     * Bao dam slew cung khong keo command vuot qua 0 sang huong nguoc.
     */
    if ((state->target_relative_angle_rad > 0.0f) &&
        (state->yaw_rate_command_rad_s < 0.0f))
    {
        state->yaw_rate_command_rad_s = 0.0f;
    }
    else if ((state->target_relative_angle_rad < 0.0f) &&
             (state->yaw_rate_command_rad_s > 0.0f))
    {
        state->yaw_rate_command_rad_s = 0.0f;
    }

    FillOutput(state, error_rad, output);
}
