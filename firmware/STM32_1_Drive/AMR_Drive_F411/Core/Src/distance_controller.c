#include "distance_controller.h"

#include <math.h>
#include <stddef.h>

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

static bool PositionIsValid(float position_m)
{
    /* So sanh cung loai NaN. Gioi han rong de bat loi bo nho/cam bien. */
    return (position_m >= -100000.0f) &&
           (position_m <= 100000.0f);
}

static bool SpeedIsValid(float speed_m_s)
{
    return (speed_m_s >= -10.0f) && (speed_m_s <= 10.0f);
}

static void FillOutput(const DistanceControllerState_t *state,
                       float remaining_distance_m,
                       DistanceControllerOutput_t *output)
{
    output->status = state->status;
    output->target_distance_m = state->target_distance_m;
    output->traveled_distance_m = state->traveled_distance_m;
    output->remaining_distance_m = remaining_distance_m;
    output->linear_speed_command_m_s =
        state->linear_speed_command_m_s;
    output->active =
        (state->status == DISTANCE_CONTROLLER_RUNNING) ||
        (state->status == DISTANCE_CONTROLLER_SETTLING);
    output->completed = (state->status == DISTANCE_CONTROLLER_DONE);
    output->fault =
        (state->status == DISTANCE_CONTROLLER_FAULT_TIMEOUT) ||
        (state->status == DISTANCE_CONTROLLER_FAULT_MEASUREMENT) ||
        (state->status == DISTANCE_CONTROLLER_FAULT_ARGUMENT);
}

void DistanceController_Init(DistanceControllerState_t *state)
{
    DistanceController_Cancel(state);
}

void DistanceController_Cancel(DistanceControllerState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->status = DISTANCE_CONTROLLER_IDLE;
    state->target_distance_m = 0.0f;
    state->start_left_position_m = 0.0f;
    state->start_right_position_m = 0.0f;
    state->traveled_distance_m = 0.0f;
    state->linear_speed_command_m_s = 0.0f;
    state->max_linear_speed_m_s = DISTANCE_CONTROLLER_MAX_SPEED_M_S;
    state->started_at_ms = 0U;
    state->timeout_ms = 0U;
    state->settling_since_ms = 0U;
    state->settling_timer_active = false;
}

bool DistanceController_Start(DistanceControllerState_t *state,
                              float target_distance_m,
                              float max_linear_speed_m_s,
                              uint32_t timeout_ms,
                              float current_left_position_m,
                              float current_right_position_m,
                              uint32_t now_ms)
{
    if (state == NULL)
    {
        return false;
    }

    DistanceController_Cancel(state);

    const float target_magnitude_m = AbsFloat(target_distance_m);
    const bool argument_valid =
        (target_magnitude_m >= DISTANCE_CONTROLLER_MIN_TARGET_M) &&
        (target_magnitude_m <= DISTANCE_CONTROLLER_MAX_TARGET_M) &&
        (max_linear_speed_m_s >= DISTANCE_CONTROLLER_MIN_SPEED_M_S) &&
        (max_linear_speed_m_s <= DISTANCE_CONTROLLER_MAX_SPEED_M_S) &&
        (timeout_ms >= DISTANCE_CONTROLLER_MIN_TIMEOUT_MS) &&
        (timeout_ms <= DISTANCE_CONTROLLER_MAX_TIMEOUT_MS) &&
        PositionIsValid(current_left_position_m) &&
        PositionIsValid(current_right_position_m);

    if (!argument_valid)
    {
        state->status = DISTANCE_CONTROLLER_FAULT_ARGUMENT;
        return false;
    }

    state->status = DISTANCE_CONTROLLER_RUNNING;
    state->target_distance_m = target_distance_m;
    state->start_left_position_m = current_left_position_m;
    state->start_right_position_m = current_right_position_m;
    state->traveled_distance_m = 0.0f;
    state->linear_speed_command_m_s = 0.0f;
    state->max_linear_speed_m_s = max_linear_speed_m_s;
    state->started_at_ms = now_ms;
    state->timeout_ms = timeout_ms;
    return true;
}

void DistanceController_Update(DistanceControllerState_t *state,
                               float left_position_m,
                               float right_position_m,
                               float left_speed_m_s,
                               float right_speed_m_s,
                               float period_s,
                               uint32_t now_ms,
                               DistanceControllerOutput_t *output)
{
    if ((state == NULL) || (output == NULL))
    {
        return;
    }

    if ((state->status != DISTANCE_CONTROLLER_RUNNING) &&
        (state->status != DISTANCE_CONTROLLER_SETTLING))
    {
        FillOutput(state,
                   state->target_distance_m -
                       state->traveled_distance_m,
                   output);
        return;
    }

    if (!PositionIsValid(left_position_m) ||
        !PositionIsValid(right_position_m) ||
        !SpeedIsValid(left_speed_m_s) ||
        !SpeedIsValid(right_speed_m_s) ||
        !((period_s > 0.0f) && (period_s <= 0.1f)))
    {
        state->status = DISTANCE_CONTROLLER_FAULT_MEASUREMENT;
        state->linear_speed_command_m_s = 0.0f;
        FillOutput(state, 0.0f, output);
        return;
    }

    const float left_traveled_m =
        left_position_m - state->start_left_position_m;
    const float right_traveled_m =
        right_position_m - state->start_right_position_m;
    state->traveled_distance_m =
        0.5f * (left_traveled_m + right_traveled_m);

    const float remaining_distance_m =
        state->target_distance_m - state->traveled_distance_m;

    if ((now_ms - state->started_at_ms) >= state->timeout_ms)
    {
        state->status = DISTANCE_CONTROLLER_FAULT_TIMEOUT;
        state->linear_speed_command_m_s = 0.0f;
        FillOutput(state, remaining_distance_m, output);
        return;
    }

    if (AbsFloat(remaining_distance_m) <=
        DISTANCE_CONTROLLER_TOLERANCE_M)
    {
        const float measured_linear_speed_m_s =
            0.5f * (left_speed_m_s + right_speed_m_s);

        state->status = DISTANCE_CONTROLLER_SETTLING;
        state->linear_speed_command_m_s = 0.0f;

        if (AbsFloat(measured_linear_speed_m_s) <=
            DISTANCE_CONTROLLER_STOP_SPEED_M_S)
        {
            if (!state->settling_timer_active)
            {
                state->settling_since_ms = now_ms;
                state->settling_timer_active = true;
            }
            else if ((now_ms - state->settling_since_ms) >=
                     DISTANCE_CONTROLLER_SETTLE_TIME_MS)
            {
                state->status = DISTANCE_CONTROLLER_DONE;
            }
        }
        else
        {
            state->settling_timer_active = false;
        }

        FillOutput(state, remaining_distance_m, output);
        return;
    }

    state->status = DISTANCE_CONTROLLER_RUNNING;
    state->settling_timer_active = false;

    float braking_distance_m =
        AbsFloat(remaining_distance_m) -
        DISTANCE_CONTROLLER_TOLERANCE_M;

    if (braking_distance_m < 0.0f)
    {
        braking_distance_m = 0.0f;
    }

    float requested_speed_m_s =
        sqrtf(2.0f * DISTANCE_CONTROLLER_DECEL_M_S2 *
              braking_distance_m);

    requested_speed_m_s =
        ClampFloat(requested_speed_m_s,
                   DISTANCE_CONTROLLER_MIN_SPEED_M_S,
                   state->max_linear_speed_m_s);
    requested_speed_m_s =
        CopySignFloat(requested_speed_m_s, remaining_distance_m);

    const bool same_direction =
        (state->linear_speed_command_m_s * requested_speed_m_s) >= 0.0f;
    const bool increasing_magnitude =
        AbsFloat(requested_speed_m_s) >
        AbsFloat(state->linear_speed_command_m_s);
    const float slew_rate_m_s2 =
        (same_direction && increasing_magnitude) ?
        DISTANCE_CONTROLLER_ACCEL_M_S2 :
        DISTANCE_CONTROLLER_DECEL_M_S2;
    const float maximum_change_m_s = slew_rate_m_s2 * period_s;
    const float command_change_m_s =
        ClampFloat(requested_speed_m_s -
                       state->linear_speed_command_m_s,
                   -maximum_change_m_s,
                   maximum_change_m_s);

    /*
     * Xe AMR nay co co cau nang va banh tu do, luc ma sat tinh luc khoi
     * dong lon hon luc can khi xe da chay.  Ramp 5 ms tu 0 m/s lam PI chi
     * dua ra PWM rat nho trong nhieu chu ky: BTS co the keu nhung xe khong
     * thang duoc ma sat.  Khi bat dau mot chuyen dong moi, nap ngay muc
     * toc do toi thieu da duoc gioi han trong header.  Tu chu ky sau van
     * dung ramp gia toc/giam toc binh thuong va van ham mem luc gan dich.
     */
    if ((AbsFloat(state->linear_speed_command_m_s) <
         DISTANCE_CONTROLLER_MIN_SPEED_M_S) &&
        (AbsFloat(requested_speed_m_s) >=
         DISTANCE_CONTROLLER_MIN_SPEED_M_S))
    {
        state->linear_speed_command_m_s =
            CopySignFloat(DISTANCE_CONTROLLER_MIN_SPEED_M_S,
                          requested_speed_m_s);
    }
    else
    {
        state->linear_speed_command_m_s += command_change_m_s;
    }

    FillOutput(state, remaining_distance_m, output);
}
