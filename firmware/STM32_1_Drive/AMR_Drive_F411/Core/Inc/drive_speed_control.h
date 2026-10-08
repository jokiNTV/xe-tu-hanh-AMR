#ifndef DRIVE_SPEED_CONTROL_H
#define DRIVE_SPEED_CONTROL_H

#include <stdint.h>

/* MotorCtrlTask chay co dinh 200 Hz. */
#define DRIVE_CONTROL_PERIOD_S                  (0.005f)

/* Doi ve 0 de rollback dung vong PI/FF legacy. */
#define DRIVE_SPEED_CONTROLLER_V2_ENABLE        (1)

/*
 * Gioi han commissioning ban dau:
 * - Muc tieu toi da 4 rad/s (xap xi 0.4 m/s voi banh R = 0.1 m).
 * - PWM toi da 250/999, xap xi 25 %.
 * Chi tang cac gioi han nay sau khi PID da duoc do va hieu chinh tren xe.
 */
#define DRIVE_CONTROL_TARGET_LIMIT_RAD_S        (4.5f)
#define DRIVE_CONTROL_PWM_LIMIT                 (400)

typedef struct
{
    int16_t left_pwm;
    int16_t right_pwm;
    float left_target_rad_s;
    float right_target_rad_s;
    float left_error_rad_s;
    float right_error_rad_s;
} DriveSpeedControlOutput_t;

void DriveSpeedControl_Init(void);
void DriveSpeedControl_Reset(void);

void DriveSpeedControl_Update(float left_target_rad_s,
                              float right_target_rad_s,
                              float left_measured_rad_s,
                              float right_measured_rad_s,
                              DriveSpeedControlOutput_t *output);

#endif /* DRIVE_SPEED_CONTROL_H */
