#include "drive_command_policy.h"

#include <stddef.h>

static bool AxisDirectionReversed(int32_t current_target,
                                  int32_t incoming_target)
{
    return ((current_target > 0) && (incoming_target < 0)) ||
           ((current_target < 0) && (incoming_target > 0));
}

DriveCommandPolicy_t DriveCommandPolicy_Evaluate(
    bool current_command_active,
    const DriveCommand_t *current_command,
    const DriveCommand_t *incoming_command)
{
    DriveCommandPolicy_t policy =
    {
        .preserve_speed_controller = false,
        .reset_heading_hold = true
    };

    if (!current_command_active ||
        (current_command == NULL) ||
        (incoming_command == NULL) ||
        (current_command->command_type !=
         AMR_MOTION_COMMAND_WHEEL_SPEED) ||
        (incoming_command->command_type !=
         AMR_MOTION_COMMAND_WHEEL_SPEED))
    {
        return policy;
    }

    policy.preserve_speed_controller = true;
    policy.reset_heading_hold =
        AxisDirectionReversed(current_command->left_target_mrad_s,
                              incoming_command->left_target_mrad_s) ||
        AxisDirectionReversed(current_command->right_target_mrad_s,
                              incoming_command->right_target_mrad_s);

    return policy;
}
