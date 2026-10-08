#ifndef DRIVE_MOTOR_H
#define DRIVE_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define DRIVE_PWM_MAX 999U

bool DriveMotor_Init(void);
void DriveMotor_Set(int16_t left_pwm, int16_t right_pwm);
void DriveMotor_Stop(void);

#endif /* DRIVE_MOTOR_H */
