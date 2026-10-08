#ifndef MECHANISM_H
#define MECHANISM_H

#include <stdint.h>


typedef enum
{
    MECH_IDLE = 0,

    MECH_OPENING = 10,
    MECH_OPEN_DONE = 11,

    MECH_CLOSING = 12,
    MECH_CLOSE_DONE = 13,

    MECH_CLAMPING = 20,
    MECH_LIFTING = 21,
    MECH_PICK_DONE = 22,

    MECH_LOWERING = 30,
    MECH_RELEASING = 31,
    MECH_DROP_DONE = 32,

    /* V2.10 manual commissioning jog states. */
    MECH_JOG_CLOSING = 40,
    MECH_JOG_OPENING = 41,
    MECH_JOG_LIFT_UP = 42,
    MECH_JOG_LIFT_DOWN = 43,

    /* Safe fault-recovery state. Fault remains latched while opening. */
    MECH_RECOVER_OPENING = 90,

    MECH_FAULT = 99

} MechanismState_t;


typedef enum
{
    MECH_FAULT_NONE = 0,

    MECH_FAULT_OVER_FORCE = 1,
    MECH_FAULT_CLAMP_TIMEOUT = 2,

    MECH_FAULT_HCSR04 = 3,
    MECH_FAULT_LIFT_TIMEOUT = 4,
    MECH_FAULT_LOWER_TIMEOUT = 5,

    MECH_FAULT_LOAD_SLIP = 6

} MechanismFault_t;


/* INIT */
void Mechanism_Init(void);

/* LOOP */
void Mechanism_Task(void);


/* ================= COMMAND ================= */

void Mechanism_StartOpen(void);
void Mechanism_StartClose(void);
void Mechanism_StartPick(void);
void Mechanism_StartDrop(void);

void Mechanism_Stop(void);
void Mechanism_ResetFault(void);

/* V2.10 manual commissioning jog.
 * These intentionally bypass clamp position/full-travel guards.
 * Fault and active-motion interlocks are still respected.
 */
uint8_t Mechanism_StartJogClose(uint32_t duration_ms);
uint8_t Mechanism_StartJogOpen(uint32_t duration_ms);
uint8_t Mechanism_StartJogLiftUp(uint32_t duration_ms);
uint8_t Mechanism_StartJogLiftDown(uint32_t duration_ms);

/*
 * Safe recovery path for clamp-related faults only.
 * Returns 1 when the recovery motion was accepted, otherwise 0.
 * Supported faults: OVER_FORCE, CLAMP_TIMEOUT.
 */
uint8_t Mechanism_CanRecoverOpen(void);
uint8_t Mechanism_StartRecoverOpen(void);

/*
 * V2.3 TEST BUILD ONLY.
 * Injects a CLAMP_TIMEOUT fault without driving any motor.
 * Accepted only from MECH_IDLE with no active fault/busy state.
 * Remove this hook after recovery validation is complete.
 */
uint8_t Mechanism_TestInjectClampFault(void);

/* ================= CLAMP TRAVEL / SAFE RECOVERY V2.9 ================= */
void Mechanism_ClampPositionSetOpenFull(void);
void Mechanism_ClampPositionSetClosedFull(void);
void Mechanism_ClampPositionInvalidate(void);
uint8_t Mechanism_ClampPositionValid(void);
uint16_t Mechanism_GetClampPositionPermille(void);
uint32_t Mechanism_GetRecoverOpenPlannedMs(void);


/* ================= STATUS ================= */

MechanismState_t Mechanism_GetState(void);
MechanismFault_t Mechanism_GetFault(void);

uint32_t Mechanism_GetFSR(void);
float Mechanism_GetDistance(void);

uint8_t Mechanism_IsBusy(void);

uint8_t Mechanism_OpenDone(void);
uint8_t Mechanism_CloseDone(void);
uint8_t Mechanism_PickDone(void);
uint8_t Mechanism_DropDone(void);


#endif
