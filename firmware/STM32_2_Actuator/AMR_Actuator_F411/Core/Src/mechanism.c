#include "mechanism.h"

#include "actuator_driver.h"
#include "fsr_sensor.h"
#include "hcsr04.h"
#include "mechanism_config.h"

#include "stm32f4xx_hal.h"


/* =========================================================
 * STATE
 * ========================================================= */

static MechanismState_t s_state = MECH_IDLE;
static MechanismFault_t s_fault = MECH_FAULT_NONE;


/* =========================================================
 * STATUS FLAGS
 * ========================================================= */

static uint8_t s_busy = 0;

static uint8_t s_open_done = 0;
static uint8_t s_pick_done = 0;
static uint8_t s_drop_done = 0;


/* =========================================================
 * TIMERS
 * ========================================================= */

static uint32_t s_state_start_ms = 0;

static uint32_t s_fsr_ok_since_ms = 0;

static uint32_t s_force_low_since_ms = 0;

static uint32_t s_hcsr_bad_since_ms = 0;

static uint32_t s_last_fsr_ms = 0;

static uint32_t s_last_hcsr_ms = 0;

static uint8_t s_close_done = 0;

/* V2.10 manual commissioning jog duration. */
static uint32_t s_jog_duration_ms = 0U;

/* =========================================================
 * V2.9 SAFE CLAMP TRAVEL ESTIMATOR
 * 0 = OPEN, 1000000 = CLOSED.
 * ========================================================= */
#define CLAMP_POS_SCALE 1000000UL
static uint32_t s_clamp_pos_q = 0U;
static uint8_t  s_clamp_pos_valid = 0U;
static uint32_t s_clamp_motion_last_ms = 0U;
static uint32_t s_clamp_motion_remainder = 0U;

static void ClampPosition_SetOpenFullInternal(void)
{
    s_clamp_pos_q = 0U;
    s_clamp_pos_valid = 1U;
    s_clamp_motion_last_ms = HAL_GetTick();
    s_clamp_motion_remainder = 0U;
}

static void ClampPosition_SetClosedFullInternal(void)
{
    s_clamp_pos_q = CLAMP_POS_SCALE;
    s_clamp_pos_valid = 1U;
    s_clamp_motion_last_ms = HAL_GetTick();
    s_clamp_motion_remainder = 0U;
}

static void ClampPosition_Update(uint32_t now)
{
    uint32_t dt;
    uint32_t delta;
    uint32_t divisor;
    uint64_t numerator;

    dt = now - s_clamp_motion_last_ms;
    s_clamp_motion_last_ms = now;
    if ((!s_clamp_pos_valid) || (dt == 0U)) return;

    if ((s_state == MECH_CLOSING) || (s_state == MECH_CLAMPING))
    {
        divisor = CLAMP_CLOSE_FULL_MS;
        numerator = ((uint64_t)dt * (uint64_t)CLAMP_POS_SCALE) +
                    (uint64_t)s_clamp_motion_remainder;
        delta = (uint32_t)(numerator / (uint64_t)divisor);
        s_clamp_motion_remainder = (uint32_t)(numerator % (uint64_t)divisor);

        if (delta >= (CLAMP_POS_SCALE - s_clamp_pos_q))
        {
            s_clamp_pos_q = CLAMP_POS_SCALE;
            s_clamp_motion_remainder = 0U;
        }
        else
        {
            s_clamp_pos_q += delta;
        }
    }
    else if ((s_state == MECH_OPENING) ||
             (s_state == MECH_RECOVER_OPENING) ||
             (s_state == MECH_RELEASING))
    {
        divisor = CLAMP_OPEN_FULL_MS;
        numerator = ((uint64_t)dt * (uint64_t)CLAMP_POS_SCALE) +
                    (uint64_t)s_clamp_motion_remainder;
        delta = (uint32_t)(numerator / (uint64_t)divisor);
        s_clamp_motion_remainder = (uint32_t)(numerator % (uint64_t)divisor);

        if (delta >= s_clamp_pos_q)
        {
            s_clamp_pos_q = 0U;
            s_clamp_motion_remainder = 0U;
        }
        else
        {
            s_clamp_pos_q -= delta;
        }
    }
}
/* =========================================================
 * INTERNAL
 * ========================================================= */

static void SetState(MechanismState_t state)
{
    s_state = state;
    s_state_start_ms = HAL_GetTick();
    s_clamp_motion_last_ms = s_state_start_ms;
    s_clamp_motion_remainder = 0U;
}


static void SetFault(MechanismFault_t fault)
{
    Actuator_StopAll();

    s_busy = 0;
    s_fault = fault;

    SetState(MECH_FAULT);
}


/* =========================================================
 * INIT
 * ========================================================= */
void Mechanism_StartClose(void)
{
    if (s_fault != MECH_FAULT_NONE)
        return;

    Actuator_StopAll();

    s_close_done = 0;
    s_busy = 1;

    SetState(MECH_CLOSING);
}

void Mechanism_Init(void)
{
    Actuator_StopAll();

    s_state = MECH_IDLE;
    s_fault = MECH_FAULT_NONE;

    s_busy = 0;

    s_open_done = 0;
    s_close_done = 0;
    s_pick_done = 0;
    s_drop_done = 0;

    s_state_start_ms = HAL_GetTick();

    s_fsr_ok_since_ms = 0;
    s_force_low_since_ms = 0;
    s_hcsr_bad_since_ms = 0;

    s_last_fsr_ms = 0;
    s_last_hcsr_ms = 0;
    s_clamp_pos_q = 0U;
    s_clamp_pos_valid = 0U;
    s_clamp_motion_last_ms = HAL_GetTick();
    s_clamp_motion_remainder = 0U;
}




/* =========================================================
 * V2.10 MANUAL COMMISSIONING JOG
 *
 * Deliberately independent of OPEN_FULL/CLOSED_FULL,
 * CLOSE_RESUME and the V2.9 clamp position estimator.
 * Intended for operator-controlled force/geometry setup only.
 * ========================================================= */
uint8_t Mechanism_StartJogClose(uint32_t duration_ms)
{
    if (s_fault != MECH_FAULT_NONE) return 0U;
    if (s_busy != 0U) return 0U;
    if ((duration_ms == 0U) || (duration_ms > 2000U)) return 0U;

    Actuator_StopAll();
    s_jog_duration_ms = duration_ms;
    s_busy = 1U;
    SetState(MECH_JOG_CLOSING);
    return 1U;
}

uint8_t Mechanism_StartJogOpen(uint32_t duration_ms)
{
    if (s_fault != MECH_FAULT_NONE) return 0U;
    if (s_busy != 0U) return 0U;
    if ((duration_ms == 0U) || (duration_ms > 2000U)) return 0U;

    Actuator_StopAll();
    s_jog_duration_ms = duration_ms;
    s_busy = 1U;
    SetState(MECH_JOG_OPENING);
    return 1U;
}


/* =========================================================
 * MANUAL LIFT CONTROL - TIME BASED, NO HC-SR04
 *
 * Current physical configuration has no distance sensor.
 * These commands are intentionally independent of PICK/DROP.
 * ========================================================= */
uint8_t Mechanism_StartJogLiftUp(uint32_t duration_ms)
{
    if (s_fault != MECH_FAULT_NONE) return 0U;
    if (s_busy != 0U) return 0U;
    if ((duration_ms == 0U) || (duration_ms > LIFT_UP_TIME_MS)) return 0U;

    Actuator_StopAll();
    s_jog_duration_ms = duration_ms;
    s_busy = 1U;
    SetState(MECH_JOG_LIFT_UP);
    return 1U;
}

uint8_t Mechanism_StartJogLiftDown(uint32_t duration_ms)
{
    if (s_fault != MECH_FAULT_NONE) return 0U;
    if (s_busy != 0U) return 0U;
    if ((duration_ms == 0U) || (duration_ms > LIFT_DOWN_FULL_TIME_MS)) return 0U;

    Actuator_StopAll();
    s_jog_duration_ms = duration_ms;
    s_busy = 1U;
    SetState(MECH_JOG_LIFT_DOWN);
    return 1U;
}

/* =========================================================
 * COMMAND: OPEN
 *
 * Dùng trước khi AMR tiến vào lấy thùng.
 * ========================================================= */

void Mechanism_StartOpen(void)
{
    if (s_fault != MECH_FAULT_NONE)
        return;

    Actuator_StopAll();

    s_open_done = 0;
    s_pick_done = 0;
    s_drop_done = 0;

    s_busy = 1;

    SetState(MECH_OPENING);
}


/* =========================================================
 * COMMAND: PICK
 *
 * CLOSE
 * -> FSR đủ lực
 * -> STOP CLAMP
 * -> LIFT
 * -> HC-SR04 TOP
 * -> PICK_DONE
 * ========================================================= */

void Mechanism_StartPick(void)
{
    if (s_fault != MECH_FAULT_NONE)
        return;

    Actuator_StopAll();

    s_open_done = 0;
    s_pick_done = 0;
    s_drop_done = 0;

    s_fsr_ok_since_ms = 0;
    s_force_low_since_ms = 0;
    s_hcsr_bad_since_ms = 0;

    s_busy = 1;

    SetState(MECH_CLAMPING);
}


/* =========================================================
 * COMMAND: DROP
 *
 * LOWER
 * -> HC-SR04 BOTTOM
 * -> STOP LIFT
 * -> OPEN CLAMP
 * -> DROP_DONE
 * ========================================================= */

void Mechanism_StartDrop(void)
{
    if (s_fault != MECH_FAULT_NONE)
        return;

    Actuator_StopAll();

    s_drop_done = 0;

    s_hcsr_bad_since_ms = 0;

    s_busy = 1;

    SetState(MECH_LOWERING);
}


/* =========================================================
 * STOP
 * ========================================================= */

void Mechanism_Stop(void)
{
    ClampPosition_Update(HAL_GetTick());
    Actuator_StopAll();

    s_busy = 0;

    /*
     * V2_RECOVERY: do not create the ambiguous state IDLE + FAULT.
     * If a fault is latched, STOP keeps the mechanism in MECH_FAULT.
     * STOP never clears a fault.
     */
    if (s_fault != MECH_FAULT_NONE)
    {
        SetState(MECH_FAULT);
    }
    else
    {
        SetState(MECH_IDLE);
    }
}


/* =========================================================
 * RESET FAULT
 * ========================================================= */

void Mechanism_ResetFault(void)
{
    Actuator_StopAll();

    s_fault = MECH_FAULT_NONE;

    s_busy = 0;

    s_open_done = 0;
    s_close_done = 0;
    s_pick_done = 0;
    s_drop_done = 0;

    s_fsr_ok_since_ms = 0;
    s_force_low_since_ms = 0;
    s_hcsr_bad_since_ms = 0;

    SetState(MECH_IDLE);
}


/* =========================================================
 * SAFE RECOVERY: OPEN CLAMP
 *
 * Only clamp-related faults are eligible.  The original fault
 * stays latched while the clamp is opening.  If STOP is issued,
 * the fault remains latched.  The fault is cleared only after the
 * timed open motion completes.
 * ========================================================= */

uint8_t Mechanism_CanRecoverOpen(void)
{
    if (s_state != MECH_FAULT) return 0U;
    if (!s_clamp_pos_valid) return 0U;
    if ((s_fault == MECH_FAULT_OVER_FORCE) ||
        (s_fault == MECH_FAULT_CLAMP_TIMEOUT)) return 1U;
    return 0U;
}


uint8_t Mechanism_StartRecoverOpen(void)
{
    if (!Mechanism_CanRecoverOpen()) return 0U;
    Actuator_StopAll();
    s_open_done = 0; s_close_done = 0; s_pick_done = 0; s_drop_done = 0;
    s_fsr_ok_since_ms = 0; s_force_low_since_ms = 0; s_hcsr_bad_since_ms = 0;
    if (s_clamp_pos_q == 0U)
    {
        s_fault = MECH_FAULT_NONE;
        s_busy = 0U;
        s_open_done = 1U;
        SetState(MECH_OPEN_DONE);
        return 1U;
    }
    s_busy = 1U;
    SetState(MECH_RECOVER_OPENING);
    return 1U;
}


/* =========================================================
 * V2.3 TEST-ONLY FAULT INJECTION
 *
 * Creates the exact clamp-related FAULT latch used by the real
 * CLAMP_TIMEOUT path, but never energizes a motor.  This exists
 * only to validate MiniPC -> FAULT -> RECOVER_OPEN behavior
 * without mechanically jamming the clamp.
 * ========================================================= */
uint8_t Mechanism_TestInjectClampFault(void)
{
    if (s_state != MECH_IDLE)
        return 0U;

    if (s_fault != MECH_FAULT_NONE)
        return 0U;

    if (s_busy != 0U)
        return 0U;

    Actuator_StopAll();

    s_open_done = 0;
    s_close_done = 0;
    s_pick_done = 0;
    s_drop_done = 0;

    s_fsr_ok_since_ms = 0;
    s_force_low_since_ms = 0;
    s_hcsr_bad_since_ms = 0;

    s_busy = 0U;
    s_fault = MECH_FAULT_CLAMP_TIMEOUT;
    SetState(MECH_FAULT);

    return 1U;
}


/* =========================================================
 * SENSOR TASK
 * ========================================================= */

static void SensorTask(uint32_t now)
{
    /* FSR khoảng 50 Hz */
    if ((now - s_last_fsr_ms) >= 20U)
    {
        s_last_fsr_ms = now;

        FSR_Update();
    }


    /* HC-SR04 khoảng 12.5 Hz */
    if ((now - s_last_hcsr_ms) >= 80U)
    {
        s_last_hcsr_ms = now;

        HCSR04_Update();
    }
}


/* =========================================================
 * HC-SR04 SAFETY
 * ========================================================= */

static uint8_t CheckHCSR04(uint32_t now)
{
    if (HCSR04_IsValid())
    {
        s_hcsr_bad_since_ms = 0;

        return 1;
    }


    if (s_hcsr_bad_since_ms == 0)
    {
        s_hcsr_bad_since_ms = now;
    }


    /*
     * Cho phép mất tín hiệu ngắn.
     * Nếu mất > 500 ms -> FAULT.
     */
    if ((now - s_hcsr_bad_since_ms) >= 500U)
    {
        SetFault(MECH_FAULT_HCSR04);
    }


    return 0;
}


/* =========================================================
 * MAIN STATE MACHINE
 * ========================================================= */

void Mechanism_Task(void)
{
    uint32_t now = HAL_GetTick();

    ClampPosition_Update(now);
    SensorTask(now);


    /* =====================================================
     * V2.10 MANUAL COMMISSIONING JOG
     * No position estimator/end-state guard is consulted here.
     * The operator advances in short timed pulses.
     * ===================================================== */
    if (s_state == MECH_JOG_CLOSING)
    {
        s_busy = 1U;
        Clamp_Close(CLAMP_CLOSE_PWM);

        if ((now - s_state_start_ms) >= s_jog_duration_ms)
        {
            Clamp_Stop();
            s_busy = 0U;
            SetState(MECH_IDLE);
        }
        return;
    }

    if (s_state == MECH_JOG_OPENING)
    {
        s_busy = 1U;
        Clamp_Open(CLAMP_OPEN_PWM);

        if ((now - s_state_start_ms) >= s_jog_duration_ms)
        {
            Clamp_Stop();
            s_busy = 0U;
            SetState(MECH_IDLE);
        }
        return;
    }


    if (s_state == MECH_JOG_LIFT_UP)
    {
        s_busy = 1U;

        if ((now - s_state_start_ms) >= s_jog_duration_ms)
        {
            Lift_Stop();
            s_busy = 0U;
            SetState(MECH_IDLE);
            return;
        }

        Lift_Up(LIFT_PWM_DEFAULT);
        return;
    }

    if (s_state == MECH_JOG_LIFT_DOWN)
    {
        s_busy = 1U;

        if ((now - s_state_start_ms) >= s_jog_duration_ms)
        {
            Lift_Stop();
            s_busy = 0U;
            SetState(MECH_IDLE);
            return;
        }

        Lift_Down(LIFT_PWM_DEFAULT);
        return;
    }

    if (s_state == MECH_CLOSING)
    {
        s_busy = 1;

        Clamp_Close(CLAMP_CLOSE_PWM);

        if (((s_clamp_pos_valid) && (s_clamp_pos_q >= CLAMP_POS_SCALE)) ||
            ((!s_clamp_pos_valid) && ((now - s_state_start_ms) >= CLAMP_CLOSE_FULL_MS)))
        {
            Clamp_Stop();
            s_busy = 0;
            s_close_done = 1;
            ClampPosition_SetClosedFullInternal();
            SetState(MECH_CLOSE_DONE);
            return;
        }

        return;
    }

    if (s_state == MECH_CLOSE_DONE)
    {
        Clamp_Stop();

        s_busy = 0;
        s_close_done = 1;

        return;
    }
    /* =====================================================
     * SAFE RECOVERY: OPEN CLAMP AFTER CLAMP-RELATED FAULT
     * ===================================================== */

    if (s_state == MECH_RECOVER_OPENING)
    {
        s_busy = 1;
        Clamp_Open(CLAMP_OPEN_PWM);

        if (s_clamp_pos_valid && (s_clamp_pos_q == 0U))
        {
            Clamp_Stop();
            s_busy = 0;
            s_open_done = 1;
            ClampPosition_SetOpenFullInternal();
            s_fault = MECH_FAULT_NONE;
            s_fsr_ok_since_ms = 0;
            s_force_low_since_ms = 0;
            s_hcsr_bad_since_ms = 0;
            SetState(MECH_OPEN_DONE);
            return;
        }

        if ((now - s_state_start_ms) >= CLAMP_OPEN_FULL_MS)
        {
            Clamp_Stop();
            s_busy = 0;
            SetState(MECH_FAULT);
            return;
        }
        return;
    }


    /* =====================================================
     * IDLE
     * ===================================================== */

    if (s_state == MECH_IDLE)
    {
        Actuator_StopAll();

        return;
    }


    /* =====================================================
     * OPENING
     * ===================================================== */

    if (s_state == MECH_OPENING)
    {
        s_busy = 1;

        Clamp_Open(CLAMP_OPEN_PWM);

        if (((s_clamp_pos_valid) && (s_clamp_pos_q == 0U)) ||
            ((!s_clamp_pos_valid) && ((now - s_state_start_ms) >= CLAMP_OPEN_FULL_MS)))
        {
            Clamp_Stop();
            s_busy = 0;
            s_open_done = 1;
            ClampPosition_SetOpenFullInternal();
            SetState(MECH_OPEN_DONE);
            return;
        }



        return;
    }


    /* =====================================================
     * OPEN DONE
     * ===================================================== */

    if (s_state == MECH_OPEN_DONE)
    {
        Clamp_Stop();

        s_busy = 0;

        return;
    }


    /* =====================================================
     * V2.11 SIMPLE PICK
     *
     * CLAMPING:
     *   close until FSR >= FSR_CLAMP_OK_ADC, then stop clamp.
     * LIFTING:
     *   lift until HC-SR04 <= 3.0 cm, then stop lift.
     *
     * No OPEN_FULL/CLOSED_FULL/POS_PERMILLE/ARM gate is used
     * inside this PICK motion path.
     * ===================================================== */

    if (s_state == MECH_CLAMPING)
    {
        s_busy = 1U;
        Clamp_Close(CLAMP_CLOSE_PWM);

        if (FSR_GetFiltered() >= FSR_CLAMP_OK_ADC)
        {
            Clamp_Stop();
            s_hcsr_bad_since_ms = 0U;
            SetState(MECH_LIFTING);
            return;
        }

        return;
    }

    if (s_state == MECH_LIFTING)
    {
        s_busy = 1U;

        /*
         * HC-SR04 has been removed from the current mechanism.
         * Lift for the calibrated fixed duration, then stop.
         */
        if ((now - s_state_start_ms) >= LIFT_UP_TIME_MS)
        {
            Lift_Stop();
            s_busy = 0U;
            s_pick_done = 1U;
            SetState(MECH_PICK_DONE);
            return;
        }

        Lift_Up(LIFT_PWM_DEFAULT);
        return;
    }


    /* =====================================================
     * PICK DONE
     * ===================================================== */

    if (s_state == MECH_PICK_DONE)
    {
        /*
         * Không cấp điện liên tục cho motor.
         * Cơ cấu tự giữ vị trí.
         */
        Lift_Stop();
        Clamp_Stop();

        s_busy = 0;
        s_pick_done = 1;

        return;
    }


    /* =====================================================
     * DROP: LOWERING
     * ===================================================== */

    if (s_state == MECH_LOWERING)
    {
        s_busy = 1U;

        /*
         * HC-SR04 has been removed from the current mechanism.
         * Lower for the calibrated full-down duration, then stop
         * and continue the existing DROP release sequence.
         */
        if ((now - s_state_start_ms) >= LIFT_DOWN_FULL_TIME_MS)
        {
            Lift_Stop();
            SetState(MECH_RELEASING);
            return;
        }

        Lift_Down(LIFT_PWM_DEFAULT);
        return;
    }


    /* =====================================================
     * DROP: RELEASE
     * ===================================================== */

    if (s_state == MECH_RELEASING)
    {
        s_busy = 1;


        /* Mở má kẹp */
        Clamp_Open(CLAMP_OPEN_PWM);

        if (((s_clamp_pos_valid) && (s_clamp_pos_q == 0U)) ||
            ((!s_clamp_pos_valid) && ((now - s_state_start_ms) >= CLAMP_OPEN_FULL_MS)))
        {
            Clamp_Stop();
            s_busy = 0;
            s_drop_done = 1;
            ClampPosition_SetOpenFullInternal();
            SetState(MECH_DROP_DONE);
            return;
        }



        return;
    }


    /* =====================================================
     * DROP DONE
     * ===================================================== */

    if (s_state == MECH_DROP_DONE)
    {
        Actuator_StopAll();

        s_busy = 0;
        s_drop_done = 1;

        return;
    }


    /* =====================================================
     * FAULT
     * ===================================================== */

    if (s_state == MECH_FAULT)
    {
        Actuator_StopAll();

        s_busy = 0;

        return;
    }


    /* =====================================================
     * UNKNOWN STATE -> FAIL SAFE
     * ===================================================== */

    Actuator_StopAll();

    s_busy = 0;
}


/* =========================================================
 * V2.9 CLAMP POSITION / SAFE RECOVERY API
 * ========================================================= */
void Mechanism_ClampPositionSetOpenFull(void) { ClampPosition_SetOpenFullInternal(); }
void Mechanism_ClampPositionSetClosedFull(void) { ClampPosition_SetClosedFullInternal(); }
void Mechanism_ClampPositionInvalidate(void)
{
    s_clamp_pos_valid = 0U;
    s_clamp_motion_last_ms = HAL_GetTick();
    s_clamp_motion_remainder = 0U;
}
uint8_t Mechanism_ClampPositionValid(void) { return s_clamp_pos_valid; }
uint16_t Mechanism_GetClampPositionPermille(void)
{
    uint32_t value;
    if (!s_clamp_pos_valid) return 0U;
    value = (s_clamp_pos_q * 1000UL) / CLAMP_POS_SCALE;
    if (value > 1000UL) value = 1000UL;
    return (uint16_t)value;
}
uint32_t Mechanism_GetRecoverOpenPlannedMs(void)
{
    uint64_t scaled; uint32_t ms;
    if (!s_clamp_pos_valid) return 0U;
    scaled = ((uint64_t)s_clamp_pos_q * (uint64_t)CLAMP_OPEN_FULL_MS);
    ms = (uint32_t)((scaled + (uint64_t)CLAMP_POS_SCALE - 1ULL) / (uint64_t)CLAMP_POS_SCALE);
    if (ms > CLAMP_OPEN_FULL_MS) ms = CLAMP_OPEN_FULL_MS;
    return ms;
}

/* =========================================================
 * STATUS GETTERS
 * ========================================================= */

MechanismState_t Mechanism_GetState(void)
{
    return s_state;
}


MechanismFault_t Mechanism_GetFault(void)
{
    return s_fault;
}


uint32_t Mechanism_GetFSR(void)
{
    return FSR_GetFiltered();
}


float Mechanism_GetDistance(void)
{
    return HCSR04_GetDistanceCm();
}


uint8_t Mechanism_IsBusy(void)
{
    return s_busy;
}


uint8_t Mechanism_OpenDone(void)
{
    return s_open_done;
}


uint8_t Mechanism_PickDone(void)
{
    return s_pick_done;
}


uint8_t Mechanism_DropDone(void)
{
    return s_drop_done;
}
uint8_t Mechanism_CloseDone(void)
{
    return s_close_done;
}
