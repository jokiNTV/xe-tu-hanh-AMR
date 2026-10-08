#ifndef DRIVE_MOTION_PROFILE_H
#define DRIVE_MOTION_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Startup motion profile for a differential-drive AMR with passive caster.
 *
 * Purpose:
 *   - During the first straight-motion segment, keep wheel speed low so the
 *     caster has time to swivel into the travel direction.
 *   - Hold the startup heading long enough to reject the lateral/yaw kick.
 *   - After the chassis is stable, ramp each wheel toward the requested
 *     command with acceleration + jerk limiting.
 *
 * Placement: BEFORE heading-hold and BEFORE wheel PI/FF.
 * Rollback: set DRIVE_MOTION_PROFILE_ENABLE to 0.
 */
#define DRIVE_MOTION_PROFILE_ENABLE                 (1)
#define DRIVE_MOTION_PROFILE_STRAIGHT_DIFF_RAD_S    (0.10f)
#define DRIVE_MOTION_PROFILE_MIN_WHEEL_RAD_S        (0.30f)
#define DRIVE_MOTION_PROFILE_ALIGN_WHEEL_RAD_S      (0.35f)
#define DRIVE_MOTION_PROFILE_ALIGN_MIN_TIME_MS      (1500U)
#define DRIVE_MOTION_PROFILE_STABLE_TIME_MS         (500U)
#define DRIVE_MOTION_PROFILE_ALIGN_TIMEOUT_MS       (4000U)
#define DRIVE_MOTION_PROFILE_YAW_TOLERANCE_RAD      (0.0087266463f) /* 0.5 deg */
#define DRIVE_MOTION_PROFILE_GYRO_TOLERANCE_RAD_S   (0.0523598776f) /* 3 deg/s */
#define DRIVE_MOTION_PROFILE_MAX_ACCEL_RAD_S2       (0.80f)
#define DRIVE_MOTION_PROFILE_MAX_DECEL_RAD_S2       (1.50f)
#define DRIVE_MOTION_PROFILE_MAX_JERK_RAD_S3        (8.00f)
#define DRIVE_MOTION_PROFILE_SPEED_DONE_RAD_S       (0.01f)

/* Keep status 4 as timeout fault to match the previous tuning notes. */
typedef enum
{
    DRIVE_MOTION_PROFILE_IDLE = 0,
    DRIVE_MOTION_PROFILE_ALIGN = 1,
    DRIVE_MOTION_PROFILE_STABILIZE = 2,
    DRIVE_MOTION_PROFILE_RAMP = 3,
    DRIVE_MOTION_PROFILE_FAULT_TIMEOUT = 4,
    DRIVE_MOTION_PROFILE_RUN = 5
} DriveMotionProfileStatus_t;

typedef struct
{
    DriveMotionProfileStatus_t status;
    float reference_yaw_rad;
    float profiled_left_rad_s;
    float profiled_right_rad_s;
    float left_accel_rad_s2;
    float right_accel_rad_s2;
    uint32_t start_ms;
    uint32_t stable_since_ms;
    bool stable_timer_active;
    bool fault_latched;
} DriveMotionProfileState_t;

typedef struct
{
    float left_target_rad_s;
    float right_target_rad_s;
    float reference_yaw_rad;
    float yaw_error_rad;
    uint32_t elapsed_ms;
    uint32_t stable_ms;
    uint8_t status;
    bool active;
    bool fault;
} DriveMotionProfileOutput_t;

void DriveMotionProfile_Init(DriveMotionProfileState_t *state);
void DriveMotionProfile_Reset(DriveMotionProfileState_t *state);

void DriveMotionProfile_Update(DriveMotionProfileState_t *state,
                               bool imu_ready,
                               float yaw_rad,
                               float gyro_z_rad_s,
                               float left_command_rad_s,
                               float right_command_rad_s,
                               float period_s,
                               uint32_t now_ms,
                               DriveMotionProfileOutput_t *output);

#endif /* DRIVE_MOTION_PROFILE_H */
