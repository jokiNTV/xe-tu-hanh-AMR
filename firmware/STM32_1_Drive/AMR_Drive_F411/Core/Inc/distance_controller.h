#ifndef DISTANCE_CONTROLLER_H
#define DISTANCE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Bo tao profile toc do de xe chay dung quang duong.
 * Don vi:
 *   quang duong, vi tri: m
 *   toc do: m/s
 *   gia toc: m/s^2
 */
#define DISTANCE_CONTROLLER_MIN_TARGET_M          (0.050f)
#define DISTANCE_CONTROLLER_MAX_TARGET_M          (5.000f)
#define DISTANCE_CONTROLLER_MIN_SPEED_M_S         (0.060f)
#define DISTANCE_CONTROLLER_MAX_SPEED_M_S         (0.400f)

#define DISTANCE_CONTROLLER_ACCEL_M_S2            (0.20f)
#define DISTANCE_CONTROLLER_DECEL_M_S2            (0.35f)
#define DISTANCE_CONTROLLER_TOLERANCE_M           (0.015f)
#define DISTANCE_CONTROLLER_STOP_SPEED_M_S        (0.025f)
#define DISTANCE_CONTROLLER_SETTLE_TIME_MS        (200U)

#define DISTANCE_CONTROLLER_MIN_TIMEOUT_MS        (1000U)
#define DISTANCE_CONTROLLER_MAX_TIMEOUT_MS        (30000U)

typedef enum
{
    DISTANCE_CONTROLLER_IDLE = 0,
    DISTANCE_CONTROLLER_RUNNING = 1,
    DISTANCE_CONTROLLER_SETTLING = 2,
    DISTANCE_CONTROLLER_DONE = 3,
    DISTANCE_CONTROLLER_FAULT_TIMEOUT = 4,
    DISTANCE_CONTROLLER_FAULT_MEASUREMENT = 5,
    DISTANCE_CONTROLLER_FAULT_ARGUMENT = 6
} DistanceControllerStatus_t;

typedef struct
{
    DistanceControllerStatus_t status;
    float target_distance_m;
    float start_left_position_m;
    float start_right_position_m;
    float traveled_distance_m;
    float linear_speed_command_m_s;
    float max_linear_speed_m_s;
    uint32_t started_at_ms;
    uint32_t timeout_ms;
    uint32_t settling_since_ms;
    bool settling_timer_active;
} DistanceControllerState_t;

typedef struct
{
    DistanceControllerStatus_t status;
    float target_distance_m;
    float traveled_distance_m;
    float remaining_distance_m;
    float linear_speed_command_m_s;
    bool active;
    bool completed;
    bool fault;
} DistanceControllerOutput_t;

void DistanceController_Init(DistanceControllerState_t *state);
void DistanceController_Cancel(DistanceControllerState_t *state);

bool DistanceController_Start(DistanceControllerState_t *state,
                              float target_distance_m,
                              float max_linear_speed_m_s,
                              uint32_t timeout_ms,
                              float current_left_position_m,
                              float current_right_position_m,
                              uint32_t now_ms);

void DistanceController_Update(DistanceControllerState_t *state,
                               float left_position_m,
                               float right_position_m,
                               float left_speed_m_s,
                               float right_speed_m_s,
                               float period_s,
                               uint32_t now_ms,
                               DistanceControllerOutput_t *output);

#endif /* DISTANCE_CONTROLLER_H */
