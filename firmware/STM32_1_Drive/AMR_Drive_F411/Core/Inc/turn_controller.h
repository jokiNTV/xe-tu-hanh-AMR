#ifndef TURN_CONTROLLER_H
#define TURN_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Vong dieu khien goc quay nam ben ngoai PI toc do tung banh.
 * Quy uoc da do tren xe:
 *   yaw duong  = quay trai
 *   yaw am     = quay phai
 *   left < 0, right > 0 = quay trai
 */
#define TURN_CONTROLLER_PI_RAD                    (3.14159265358979323846f)
#define TURN_CONTROLLER_WHEEL_RADIUS_M            (0.100f)
#define TURN_CONTROLLER_WHEEL_SEPARATION_M        (0.600f)

#define TURN_CONTROLLER_KP                        (1.80f)
#define TURN_CONTROLLER_KD                        (0.12f)
#define TURN_CONTROLLER_MAX_YAW_RATE_RAD_S         (0.50f)
#define TURN_CONTROLLER_MIN_YAW_RATE_RAD_S         (0.12f)
#define TURN_CONTROLLER_YAW_RATE_SLEW_RAD_S2       (1.50f)

/* Giam toc goc theo khoang cach con lai de giam vuot goc khi dung. */
#define TURN_CONTROLLER_BRAKE_DECEL_RAD_S2         (0.45f)

#define TURN_CONTROLLER_TOLERANCE_RAD              (0.00349065850f)
#define TURN_CONTROLLER_STOP_GYRO_LIMIT_RAD_S      (0.05235987756f)
#define TURN_CONTROLLER_SETTLE_TIME_MS             (150U)

/* V8 TERMINAL TURN HOLD - chi ap dung cho TURN_RELATIVE. */
#define TURN_CONTROLLER_FINAL_APPROACH_RAD             (0.20943951024f) /* 12.0 deg */
#define TURN_CONTROLLER_FINAL_MAX_YAW_RATE_RAD_S        (0.18f)
#define TURN_CONTROLLER_TERMINAL_HOLD_ENTER_RAD         (0.01745329252f) /* 1.0 deg */
#define TURN_CONTROLLER_TERMINAL_HOLD_KP                (1.20f)
#define TURN_CONTROLLER_TERMINAL_HOLD_KD                (0.10f)
#define TURN_CONTROLLER_TERMINAL_HOLD_MAX_YAW_RATE_RAD_S (0.080f)
#define TURN_CONTROLLER_TERMINAL_HOLD_MIN_YAW_RATE_RAD_S (0.020f)
/* V9: short breakaway boost only when the chassis is practically stationary. */
#define TURN_CONTROLLER_TERMINAL_BREAKAWAY_YAW_RATE_RAD_S (0.060f)
#define TURN_CONTROLLER_TERMINAL_BREAKAWAY_GYRO_RAD_S      (0.010f)
#define TURN_CONTROLLER_TERMINAL_HOLD_TOLERANCE_RAD     (0.00872664626f) /* 0.5 deg */
#define TURN_CONTROLLER_TERMINAL_HOLD_GYRO_LIMIT_RAD_S  (0.01745329252f) /* 1.0 deg/s */
#define TURN_CONTROLLER_TERMINAL_HOLD_SETTLE_TIME_MS    (400U)
#define TURN_CONTROLLER_TERMINAL_BIAS_DISABLE_RAD       (0.06981317008f) /* 4.0 deg */

#define TURN_CONTROLLER_MIN_TIMEOUT_MS             (1000U)
#define TURN_CONTROLLER_MAX_TIMEOUT_MS             (15000U)

typedef enum
{
    TURN_CONTROLLER_IDLE = 0,
    TURN_CONTROLLER_RUNNING = 1,
    TURN_CONTROLLER_SETTLING = 2,
    TURN_CONTROLLER_DONE = 3,
    TURN_CONTROLLER_FAULT_TIMEOUT = 4,
    TURN_CONTROLLER_FAULT_IMU = 5,
    TURN_CONTROLLER_FAULT_ARGUMENT = 6
} TurnControllerStatus_t;

typedef struct
{
    TurnControllerStatus_t status;
    float target_relative_angle_rad;
    float previous_yaw_rad;
    float traveled_angle_rad;
    float yaw_rate_command_rad_s;
    float max_yaw_rate_rad_s;
    uint32_t started_at_ms;
    uint32_t timeout_ms;
    uint32_t settling_since_ms;
    bool settling_timer_active;
} TurnControllerState_t;

typedef struct
{
    TurnControllerStatus_t status;
    float target_relative_angle_rad;
    float traveled_angle_rad;
    float error_rad;
    float yaw_rate_command_rad_s;
    float left_target_rad_s;
    float right_target_rad_s;
    bool active;
    bool completed;
    bool fault;
} TurnControllerOutput_t;

void TurnController_Init(TurnControllerState_t *state);
void TurnController_Cancel(TurnControllerState_t *state);

bool TurnController_Start(TurnControllerState_t *state,
                          float relative_angle_rad,
                          float max_yaw_rate_rad_s,
                          uint32_t timeout_ms,
                          float current_yaw_rad,
                          uint32_t now_ms);

void TurnController_Update(TurnControllerState_t *state,
                           bool imu_ready,
                           float yaw_rad,
                           float gyro_z_rad_s,
                           float period_s,
                           uint32_t now_ms,
                           TurnControllerOutput_t *output);

#endif /* TURN_CONTROLLER_H */
