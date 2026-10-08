#ifndef DRIVE_KINEMATICS_H
#define DRIVE_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

#include "drive_encoder.h"

/*
 * Gia tri da hieu chuan tu phep do thuc te:
 * - Duong kinh banh: 0.200 m.
 * - Encoder: xap xi 14000 count / 1 vong banh.
 */
#define DRIVE_WHEEL_DIAMETER_M          (0.200f)
#define DRIVE_ENCODER_LEFT_CPR          (14000.0f)
#define DRIVE_ENCODER_RIGHT_CPR         (14000.0f)

/*
 * Bo loc thong thap bac mot cho toc do.
 * MotorCtrlTask chay 200 Hz; tan so cat 10 Hz du nhanh cho dieu khien motor
 * va giam dao dong do delta_count thay doi tung mau.
 */
#define DRIVE_SPEED_FILTER_CUTOFF_HZ    (10.0f)

typedef struct
{
    float left_position_m;
    float right_position_m;

    /* Toc do chua loc, dung de kiem tra va hieu chuan. */
    float left_speed_raw_m_s;
    float right_speed_raw_m_s;

    /* Toc do da loc, se dung cho PID. */
    float left_speed_m_s;
    float right_speed_m_s;

    float left_speed_rad_s;
    float right_speed_rad_s;

    float left_speed_rpm;
    float right_speed_rpm;

    uint32_t timestamp_ms;
} DriveKinematicsSample_t;

void DriveKinematics_Init(void);
void DriveKinematics_Reset(void);
bool DriveKinematics_Update(const DriveEncoderSample_t *encoder,
                            DriveKinematicsSample_t *kinematics);

#endif /* DRIVE_KINEMATICS_H */
