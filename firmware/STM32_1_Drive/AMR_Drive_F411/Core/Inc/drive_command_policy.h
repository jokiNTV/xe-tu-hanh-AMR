#ifndef DRIVE_COMMAND_POLICY_H
#define DRIVE_COMMAND_POLICY_H

#include <stdbool.h>

#include "app_types.h"

/*
 * Phan biet mot lenh chuyen dong moi voi goi refresh cua wheel-speed.
 * Host phai gui wheel-speed dinh ky de lam moi watchdog. Cac goi refresh
 * khong duoc reset PI, PWM slew hoac moc heading, neu khong motor se khong
 * bao gio vuot qua ma sat tinh.
 */
typedef struct
{
    bool preserve_speed_controller;
    bool reset_heading_hold;
} DriveCommandPolicy_t;

DriveCommandPolicy_t DriveCommandPolicy_Evaluate(
    bool current_command_active,
    const DriveCommand_t *current_command,
    const DriveCommand_t *incoming_command);

#endif /* DRIVE_COMMAND_POLICY_H */
