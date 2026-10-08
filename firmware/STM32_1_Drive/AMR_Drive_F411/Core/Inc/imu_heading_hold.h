#ifndef IMU_HEADING_HOLD_H
#define IMU_HEADING_HOLD_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Vong giu huong nam ben ngoai PI toc do tung banh.
 * Don vi vao/ra cua muc tieu banh la rad/s.
 */
#define IMU_HEADING_HOLD_V2_ENABLE                 (1)
#define IMU_HEADING_HOLD_STRAIGHT_DIFF_RAD_S       (0.10f)
#define IMU_HEADING_HOLD_MIN_WHEEL_RAD_S           (0.30f)
#define IMU_HEADING_HOLD_ERROR_DEADBAND_RAD         (0.0015f)

/* V1 legacy: doi macro V2_ENABLE ve 0 de rollback ngay. */
#define IMU_HEADING_HOLD_KP                        (4.20f)
#define IMU_HEADING_HOLD_KD                        (1.20f)
#define IMU_HEADING_HOLD_LEGACY_DEADBAND_RAD       (0.005f)
#define IMU_HEADING_HOLD_LEGACY_CORRECTION_RAD_S   (0.60f)
#define IMU_HEADING_HOLD_LEGACY_SLEW_RAD_S2        (3.00f)

/*
 * V2 cascade theo kien truc rover/drone:
 * heading error -> yaw-rate setpoint -> PI yaw-rate + kinematic FF.
 * b/(2R) = 0.600/(2*0.100) = 3.0000000.
 */
#define IMU_HEADING_HOLD_ANGLE_KP                  (9.00f)
#define IMU_HEADING_HOLD_YAW_RATE_LIMIT_RAD_S      (0.30f)
#define IMU_HEADING_HOLD_YAW_RATE_SLEW_RAD_S2      (1.50f)
#define IMU_HEADING_HOLD_RATE_KP                   (2.80f)
#define IMU_HEADING_HOLD_RATE_KI                   (1.50f)
#define IMU_HEADING_HOLD_RATE_I_LIMIT_RAD_S        (0.25f)
#define IMU_HEADING_HOLD_KINEMATIC_FF              (3.0000000f)
#define IMU_HEADING_HOLD_CORRECTION_LIMIT_RAD_S    (1.20f)
#define IMU_HEADING_HOLD_CORRECTION_SLEW_RAD_S2    (6.00f)

typedef struct
{
    float reference_yaw_rad;
    float desired_yaw_rate_rad_s;
    float yaw_rate_integral_correction_rad_s;
    float correction_rad_s;
    bool active;
} ImuHeadingHoldState_t;

typedef struct
{
    float reference_yaw_rad;
    float error_rad;
    float desired_yaw_rate_rad_s;
    float yaw_rate_error_rad_s;
    float rate_ff_correction_rad_s;
    float rate_p_correction_rad_s;
    float rate_i_correction_rad_s;
    float correction_rad_s;
    float left_target_rad_s;
    float right_target_rad_s;
    uint8_t controller_version;
    bool correction_saturated;
    bool active;
} ImuHeadingHoldOutput_t;

void ImuHeadingHold_Init(ImuHeadingHoldState_t *state);
void ImuHeadingHold_Reset(ImuHeadingHoldState_t *state);

void ImuHeadingHold_Update(ImuHeadingHoldState_t *state,
                           bool imu_ready,
                           float yaw_rad,
                           float gyro_z_rad_s,
                           float left_command_rad_s,
                           float right_command_rad_s,
                           float wheel_target_limit_rad_s,
                           float period_s,
                           ImuHeadingHoldOutput_t *output);

#endif /* IMU_HEADING_HOLD_H */
