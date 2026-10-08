#include "app_comm.h"
#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include <string.h>
#include "app_rtos.h"
#include "app_types.h"
#include "amr/amr_messages.h"
#include "amr/amr_protocol.h"
#include "usbd_cdc_if.h"
#define COMM_MIN_TIMEOUT_MS              50U
#define COMM_MAX_TIMEOUT_MS            1000U
#define COMM_MAX_WHEEL_TARGET_MRAD_S   12000
#define COMM_TURN_MIN_TIMEOUT_MS        1000U
#define COMM_TURN_MAX_TIMEOUT_MS       15000U
#define COMM_TURN_MAX_ANGLE_MRAD        3142
#define COMM_TURN_DEFAULT_YAW_MRAD_S     500U
#define COMM_TURN_MIN_YAW_MRAD_S         120U
#define COMM_TURN_MAX_YAW_MRAD_S         700U
#define COMM_DISTANCE_MIN_TIMEOUT_MS      1000U
#define COMM_DISTANCE_MAX_TIMEOUT_MS     30000U
#define COMM_DISTANCE_MAX_ABS_MM          5000
#define COMM_DISTANCE_MIN_ABS_MM            50
#define COMM_DISTANCE_DEFAULT_SPEED_MM_S   200U
#define COMM_DISTANCE_MIN_SPEED_MM_S        60U
#define COMM_DISTANCE_MAX_SPEED_MM_S       400U
#define COMM_ODOM_MIN_PERIOD_MS             20U
#define COMM_ODOM_MAX_PERIOD_MS           1000U
#define COMM_STREAM_OPTION_DRIVE_DIAGNOSTIC (1U << 0)
#define COMM_STREAM_SUPPORTED_OPTIONS       \
    COMM_STREAM_OPTION_DRIVE_DIAGNOSTIC
#define COMM_DRIVE_DIAGNOSTIC_PAYLOAD_SIZE 112U
#define COMM_DIAG_STATUS_COMMAND_ACTIVE     (1U << 0)
#define COMM_DIAG_STATUS_HEADING_ACTIVE     (1U << 1)
#define COMM_DIAG_STATUS_IMU_READY          (1U << 2)
#define COMM_DIAG_STATUS_ODOM_VALID         (1U << 3)
enum
{
    COMM_DETAIL_NONE = 0,
    COMM_DETAIL_DUPLICATE,
    COMM_DETAIL_FORMAT,
    COMM_DETAIL_RANGE,
    COMM_DETAIL_STATE,
    COMM_DETAIL_QUEUE_FULL,
    COMM_DETAIL_SEQUENCE,
    COMM_DETAIL_UNSUPPORTED
};
static amr_stream_decoder_t decoder;
static amr_packet_t rx_packet;
static amr_packet_t tx_packet;
static uint8_t tx_wire[AMR_MAX_WIRE_FRAME_SIZE];
static volatile uint32_t last_host_activity_ms = 0U;
static volatile uint8_t current_mode = AMR_MODE_SAFE;
static uint16_t tx_sequence = 0U;
static uint16_t last_drive_sequence = 0U;
static bool have_drive_sequence = false;
static bool odometry_stream_enabled = false;
static bool drive_diagnostic_stream_enabled = false;
static bool stream_send_diagnostic_next = false;
static bool have_telemetry_snapshot = false;
static uint16_t odometry_stream_period_ms = 100U;
static uint32_t next_odometry_tx_ms = 0U;
static TelemetryData_t latest_telemetry = {0};
static int32_t FloatToScaledInt32(float value, float scale)
{
    if (!isfinite(value))
    {
        return 0;
    }
    const double scaled = (double)value * (double)scale;
    if (scaled >= (double)INT32_MAX)
    {
        return INT32_MAX;
    }
    if (scaled <= (double)INT32_MIN)
    {
        return INT32_MIN;
    }
    return (int32_t)lround(scaled);
}
static void WriteU16Le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
}
static void WriteU32Le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)((value >> 24U) & 0xFFU);
}
static void WriteI32Le(uint8_t *destination, int32_t value)
{
    WriteU32Le(destination, (uint32_t)value);
}
static bool PacketTargetsDrive(const amr_packet_t *packet)
{
    return (packet->device_id == AMR_DEVICE_DRIVE) ||
           (packet->device_id == AMR_DEVICE_BROADCAST);
}
static bool SendAck(uint16_t acknowledged_sequence,
                    bool success,
                    uint8_t detail)
{
    amr_ack_t ack =
    {
        .acknowledged_sequence = acknowledged_sequence,
        .status = success ? 0U : 1U,
        .detail = detail
    };
    memset(&tx_packet, 0, sizeof(tx_packet));
    tx_packet.device_id = AMR_DEVICE_DRIVE;
    tx_packet.message_id = success ? AMR_MSG_ACK : AMR_MSG_NACK;
    tx_packet.flags = AMR_FLAG_IS_RESPONSE;
    if (!success)
    {
        tx_packet.flags |= AMR_FLAG_ERROR;
    }
    tx_packet.sequence = ++tx_sequence;
    tx_packet.timestamp_ms = osKernelGetTickCount();
    tx_packet.payload_length = AMR_ACK_PAYLOAD_SIZE;
    if (!amr_ack_encode(&ack,
                        tx_packet.payload,
                        sizeof(tx_packet.payload)))
    {
        return false;
    }
    size_t wire_length = 0U;
    if (amr_protocol_encode(&tx_packet,
                            tx_wire,
                            sizeof(tx_wire),
                            &wire_length) != AMR_PROTOCOL_OK)
    {
        return false;
    }
    for (uint8_t attempt = 0U; attempt < 3U; ++attempt)
    {
        if (CDC_Transmit_FS(tx_wire, (uint16_t)wire_length) == USBD_OK)
        {
            return true;
        }
        osDelay(2U);
    }
    return false;
}
static void ReplyIfRequired(const amr_packet_t *packet,
                            bool success,
                            uint8_t detail)
{
    if ((packet->flags & AMR_FLAG_ACK_REQUIRED) != 0U)
    {
        (void)SendAck(packet->sequence, success, detail);
    }
}
static void StopMotionRequest(void)
{
    (void)osMessageQueueReset(DriveCmdQueueHandle);
    (void)osEventFlagsClear(
        SystemEventsHandle,
        AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);
}
static void HandleDriveCommand(const amr_packet_t *packet)
{
    if (!PacketTargetsDrive(packet))
    {
        return;
    }
    amr_drive_command_t message;
    if (!amr_drive_command_decode(packet->payload,
                                  packet->payload_length,
                                  &message))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_FORMAT);
        return;
    }
    if (have_drive_sequence)
    {
        int16_t sequence_delta =
            (int16_t)(packet->sequence - last_drive_sequence);
        if (sequence_delta == 0)
        {
            /* ACK lại nhưng không thực thi và không làm mới watchdog. */
            ReplyIfRequired(packet, true, COMM_DETAIL_DUPLICATE);
            return;
        }
        if (sequence_delta < 0)
        {
            ReplyIfRequired(packet, false, COMM_DETAIL_SEQUENCE);
            return;
        }
    }
    uint16_t timeout_ms = message.command_timeout_ms;
    if (timeout_ms == 0U)
    {
        timeout_ms = AMR_CMD_TIMEOUT_DEFAULT_MS;
    }
    if ((timeout_ms < COMM_MIN_TIMEOUT_MS) ||
        (timeout_ms > COMM_MAX_TIMEOUT_MS))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_RANGE);
        return;
    }
    if ((message.left_target_mrad_s <
         -COMM_MAX_WHEEL_TARGET_MRAD_S) ||
        (message.left_target_mrad_s >
         COMM_MAX_WHEEL_TARGET_MRAD_S) ||
        (message.right_target_mrad_s <
         -COMM_MAX_WHEEL_TARGET_MRAD_S) ||
        (message.right_target_mrad_s >
         COMM_MAX_WHEEL_TARGET_MRAD_S))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_RANGE);
        return;
    }
    uint32_t flags = osEventFlagsGet(SystemEventsHandle);
    if (((current_mode != AMR_MODE_MANUAL) &&
         (current_mode != AMR_MODE_AUTO)) ||
        ((flags & (AMR_EVT_FAULT | AMR_EVT_ESTOP)) != 0U))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_STATE);
        return;
    }
    DriveCommand_t command =
    {
        .command_type = AMR_MOTION_COMMAND_WHEEL_SPEED,
        .left_target_mrad_s = message.left_target_mrad_s,
        .right_target_mrad_s = message.right_target_mrad_s,
        .received_at_ms = osKernelGetTickCount(),
        .sequence = packet->sequence,
        .timeout_ms = timeout_ms
    };
    /*
     * Lệnh vận tốc chỉ cần giá trị mới nhất.
     * Xóa các lệnh cũ chưa được MotorCtrlTask đọc.
     */
    (void)osMessageQueueReset(DriveCmdQueueHandle);
    if (osMessageQueuePut(DriveCmdQueueHandle,
                          &command,
                          0U,
                          0U) != osOK)
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_QUEUE_FULL);
        return;
    }
    last_drive_sequence = packet->sequence;
    have_drive_sequence = true;
    (void)osEventFlagsSet(
        SystemEventsHandle,
        AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);
    ReplyIfRequired(packet, true, COMM_DETAIL_NONE);
}
static void HandleTurnCommand(const amr_packet_t *packet)
{
    if (!PacketTargetsDrive(packet))
    {
        return;
    }
    amr_turn_command_t message;
    if (!amr_turn_command_decode(packet->payload,
                                 packet->payload_length,
                                 &message))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_FORMAT);
        return;
    }
    if (have_drive_sequence)
    {
        const int16_t sequence_delta =
            (int16_t)(packet->sequence - last_drive_sequence);
        if (sequence_delta == 0)
        {
            ReplyIfRequired(packet, true, COMM_DETAIL_DUPLICATE);
            return;
        }
        if (sequence_delta < 0)
        {
            ReplyIfRequired(packet, false, COMM_DETAIL_SEQUENCE);
            return;
        }
    }
    uint16_t max_yaw_rate_mrad_s = message.max_yaw_rate_mrad_s;
    if (max_yaw_rate_mrad_s == 0U)
    {
        max_yaw_rate_mrad_s = COMM_TURN_DEFAULT_YAW_MRAD_S;
    }
    if ((message.relative_angle_mrad < -COMM_TURN_MAX_ANGLE_MRAD) ||
        (message.relative_angle_mrad > COMM_TURN_MAX_ANGLE_MRAD) ||
        (max_yaw_rate_mrad_s < COMM_TURN_MIN_YAW_MRAD_S) ||
        (max_yaw_rate_mrad_s > COMM_TURN_MAX_YAW_MRAD_S) ||
        (message.command_timeout_ms < COMM_TURN_MIN_TIMEOUT_MS) ||
        (message.command_timeout_ms > COMM_TURN_MAX_TIMEOUT_MS))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_RANGE);
        return;
    }
    const uint32_t flags = osEventFlagsGet(SystemEventsHandle);
    if (((current_mode != AMR_MODE_MANUAL) &&
         (current_mode != AMR_MODE_AUTO)) ||
        ((flags & (AMR_EVT_FAULT | AMR_EVT_ESTOP)) != 0U) ||
        ((flags & AMR_EVT_IMU_READY) == 0U))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_STATE);
        return;
    }
    const DriveCommand_t command =
    {
        .command_type = AMR_MOTION_COMMAND_TURN_RELATIVE,
        .turn_angle_mrad = message.relative_angle_mrad,
        .turn_max_yaw_rate_mrad_s = max_yaw_rate_mrad_s,
        .received_at_ms = osKernelGetTickCount(),
        .sequence = packet->sequence,
        .timeout_ms = message.command_timeout_ms,
        .command_token = message.command_token
    };
    /* Moi lệnh quay thay the moi lenh chuyen dong dang cho. */
    (void)osMessageQueueReset(DriveCmdQueueHandle);
    if (osMessageQueuePut(DriveCmdQueueHandle,
                          &command,
                          0U,
                          0U) != osOK)
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_QUEUE_FULL);
        return;
    }
    last_drive_sequence = packet->sequence;
    have_drive_sequence = true;
    (void)osEventFlagsSet(
        SystemEventsHandle,
        AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);
    ReplyIfRequired(packet, true, COMM_DETAIL_NONE);
}
static void HandleDistanceCommand(const amr_packet_t *packet)
{
    if (!PacketTargetsDrive(packet))
    {
        return;
    }
    amr_distance_command_t message;
    if (!amr_distance_command_decode(packet->payload,
                                     packet->payload_length,
                                     &message))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_FORMAT);
        return;
    }
    if (have_drive_sequence)
    {
        const int16_t sequence_delta =
            (int16_t)(packet->sequence - last_drive_sequence);
        if (sequence_delta == 0)
        {
            ReplyIfRequired(packet, true, COMM_DETAIL_DUPLICATE);
            return;
        }
        if (sequence_delta < 0)
        {
            ReplyIfRequired(packet, false, COMM_DETAIL_SEQUENCE);
            return;
        }
    }
    uint16_t max_speed_mm_s = message.max_speed_mm_s;
    if (max_speed_mm_s == 0U)
    {
        max_speed_mm_s = COMM_DISTANCE_DEFAULT_SPEED_MM_S;
    }
    const int64_t signed_distance_mm = (int64_t)message.distance_mm;
    const int64_t absolute_distance_mm =
        (signed_distance_mm < 0) ?
        -signed_distance_mm : signed_distance_mm;
    if ((absolute_distance_mm < COMM_DISTANCE_MIN_ABS_MM) ||
        (absolute_distance_mm > COMM_DISTANCE_MAX_ABS_MM) ||
        (max_speed_mm_s < COMM_DISTANCE_MIN_SPEED_MM_S) ||
        (max_speed_mm_s > COMM_DISTANCE_MAX_SPEED_MM_S) ||
        (message.command_timeout_ms < COMM_DISTANCE_MIN_TIMEOUT_MS) ||
        (message.command_timeout_ms > COMM_DISTANCE_MAX_TIMEOUT_MS))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_RANGE);
        return;
    }
    const uint32_t flags = osEventFlagsGet(SystemEventsHandle);
    if (((current_mode != AMR_MODE_MANUAL) &&
         (current_mode != AMR_MODE_AUTO)) ||
        ((flags & (AMR_EVT_FAULT | AMR_EVT_ESTOP)) != 0U) ||
        ((flags & AMR_EVT_IMU_READY) == 0U))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_STATE);
        return;
    }
    const DriveCommand_t command =
    {
        .command_type = AMR_MOTION_COMMAND_DRIVE_DISTANCE,
        .distance_mm = message.distance_mm,
        .distance_max_speed_mm_s = max_speed_mm_s,
        .received_at_ms = osKernelGetTickCount(),
        .sequence = packet->sequence,
        .timeout_ms = message.command_timeout_ms,
        .command_token = message.command_token
    };
    (void)osMessageQueueReset(DriveCmdQueueHandle);
    if (osMessageQueuePut(DriveCmdQueueHandle,
                          &command,
                          0U,
                          0U) != osOK)
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_QUEUE_FULL);
        return;
    }
    last_drive_sequence = packet->sequence;
    have_drive_sequence = true;
    (void)osEventFlagsSet(
        SystemEventsHandle,
        AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);
    ReplyIfRequired(packet, true, COMM_DETAIL_NONE);
}
static void HandleOdometryStreamControl(const amr_packet_t *packet)
{
    if (!PacketTargetsDrive(packet))
    {
        return;
    }
    amr_odometry_stream_control_t message;
    if (!amr_odometry_stream_control_decode(packet->payload,
                                             packet->payload_length,
                                             &message))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_FORMAT);
        return;
    }
    if (((message.options & ~COMM_STREAM_SUPPORTED_OPTIONS) != 0U) ||
        ((message.period_ms != 0U) &&
         ((message.period_ms < COMM_ODOM_MIN_PERIOD_MS) ||
          (message.period_ms > COMM_ODOM_MAX_PERIOD_MS))))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_RANGE);
        return;
    }
    if (message.period_ms == 0U)
    {
        odometry_stream_enabled = false;
        drive_diagnostic_stream_enabled = false;
        stream_send_diagnostic_next = false;
    }
    else
    {
        odometry_stream_period_ms = message.period_ms;
        next_odometry_tx_ms = osKernelGetTickCount();
        odometry_stream_enabled = true;
        drive_diagnostic_stream_enabled =
            (message.options &
             COMM_STREAM_OPTION_DRIVE_DIAGNOSTIC) != 0U;
        stream_send_diagnostic_next = false;
    }
    ReplyIfRequired(packet, true, COMM_DETAIL_NONE);
}
static bool SendOdometryTelemetry(const TelemetryData_t *telemetry)
{
    if (telemetry == NULL)
    {
        return false;
    }
    const amr_odometry_t message =
    {
        .sample_timestamp_ms = telemetry->timestamp_ms,
        .x_mm = FloatToScaledInt32(telemetry->odom_x_m, 1000.0f),
        .y_mm = FloatToScaledInt32(telemetry->odom_y_m, 1000.0f),
        .yaw_mrad = FloatToScaledInt32(telemetry->odom_yaw_rad,
                                      1000.0f),
        .linear_velocity_mm_s =
            FloatToScaledInt32(telemetry->linear_velocity_m_s, 1000.0f),
        .angular_velocity_mrad_s =
            FloatToScaledInt32(telemetry->angular_velocity_rad_s,
                               1000.0f),
        .sample_count = telemetry->sample_count,
        .status = (telemetry->odom_valid != 0U) ?
                  AMR_ODOMETRY_STATUS_VALID : 0U,
        .reserved = {0U, 0U, 0U}
    };
    memset(&tx_packet, 0, sizeof(tx_packet));
    tx_packet.device_id = AMR_DEVICE_DRIVE;
    tx_packet.message_id = AMR_MSG_ODOMETRY;
    tx_packet.flags = AMR_FLAG_NONE;
    tx_packet.sequence = ++tx_sequence;
    tx_packet.timestamp_ms = osKernelGetTickCount();
    tx_packet.payload_length = AMR_ODOMETRY_PAYLOAD_SIZE;
    if (!amr_odometry_encode(&message,
                             tx_packet.payload,
                             sizeof(tx_packet.payload)))
    {
        return false;
    }
    size_t wire_length = 0U;
    if (amr_protocol_encode(&tx_packet,
                            tx_wire,
                            sizeof(tx_wire),
                            &wire_length) != AMR_PROTOCOL_OK)
    {
        return false;
    }
    return CDC_Transmit_FS(tx_wire, (uint16_t)wire_length) == USBD_OK;
}
static bool SendDriveDiagnosticTelemetry(
    const TelemetryData_t *telemetry)
{
    if (telemetry == NULL)
    {
        return false;
    }
    uint8_t status = 0U;
    if (telemetry->command_active != 0U)
    {
        status |= COMM_DIAG_STATUS_COMMAND_ACTIVE;
    }
    if (telemetry->heading_hold_active != 0U)
    {
        status |= COMM_DIAG_STATUS_HEADING_ACTIVE;
    }
    if (telemetry->imu_ready != 0U)
    {
        status |= COMM_DIAG_STATUS_IMU_READY;
    }
    if (telemetry->odom_valid != 0U)
    {
        status |= COMM_DIAG_STATUS_ODOM_VALID;
    }
    memset(&tx_packet, 0, sizeof(tx_packet));
    tx_packet.device_id = AMR_DEVICE_DRIVE;
    tx_packet.message_id = AMR_MSG_DRIVE_DIAGNOSTIC;
    tx_packet.flags = AMR_FLAG_NONE;
    tx_packet.sequence = ++tx_sequence;
    tx_packet.timestamp_ms = osKernelGetTickCount();
    tx_packet.payload_length = COMM_DRIVE_DIAGNOSTIC_PAYLOAD_SIZE;
    WriteU32Le(&tx_packet.payload[0], telemetry->timestamp_ms);
    WriteI32Le(&tx_packet.payload[4],
               FloatToScaledInt32(telemetry->target_left_rad_s,
                                  1000.0f));
    WriteI32Le(&tx_packet.payload[8],
               FloatToScaledInt32(telemetry->target_right_rad_s,
                                  1000.0f));
    WriteI32Le(&tx_packet.payload[12],
               FloatToScaledInt32(telemetry->measured_left_rad_s,
                                  1000.0f));
    WriteI32Le(&tx_packet.payload[16],
               FloatToScaledInt32(telemetry->measured_right_rad_s,
                                  1000.0f));
    WriteU16Le(&tx_packet.payload[20],
               (uint16_t)telemetry->pwm_left);
    WriteU16Le(&tx_packet.payload[22],
               (uint16_t)telemetry->pwm_right);
    WriteI32Le(&tx_packet.payload[24],
               FloatToScaledInt32(telemetry->imu_yaw_rad, 1000.0f));
    WriteI32Le(&tx_packet.payload[28],
               FloatToScaledInt32(telemetry->gyro_z_rad_s, 1000.0f));
    WriteI32Le(&tx_packet.payload[32],
               FloatToScaledInt32(telemetry->heading_reference_rad,
                                  1000.0f));
    WriteI32Le(&tx_packet.payload[36],
               FloatToScaledInt32(telemetry->heading_error_rad,
                                  1000.0f));
    WriteI32Le(&tx_packet.payload[40],
               FloatToScaledInt32(
                   telemetry->heading_correction_rad_s,
                   1000.0f));
    WriteI32Le(&tx_packet.payload[44],
               FloatToScaledInt32(
                   telemetry->encoder_yaw_rate_rad_s,
                   1000.0f));
    WriteI32Le(&tx_packet.payload[48],
               FloatToScaledInt32(
                   telemetry->yaw_rate_residual_rad_s,
                   1000.0f));
    WriteI32Le(&tx_packet.payload[52],
               FloatToScaledInt32(telemetry->accel_x_m_s2, 1000.0f));
    WriteI32Le(&tx_packet.payload[56],
               FloatToScaledInt32(telemetry->accel_y_m_s2, 1000.0f));
    WriteI32Le(&tx_packet.payload[60],
               FloatToScaledInt32(telemetry->accel_z_m_s2, 1000.0f));
    WriteI32Le(&tx_packet.payload[64],
               FloatToScaledInt32(telemetry->odom_x_m, 1000.0f));
    WriteI32Le(&tx_packet.payload[68],
               FloatToScaledInt32(telemetry->odom_y_m, 1000.0f));
    WriteI32Le(&tx_packet.payload[72],
               FloatToScaledInt32(telemetry->odom_yaw_rad, 1000.0f));
    WriteU32Le(&tx_packet.payload[76],
               telemetry->command_refresh_count);
    WriteU16Le(&tx_packet.payload[80], telemetry->command_age_ms);
    tx_packet.payload[82] = status;
    tx_packet.payload[83] = telemetry->mode;
    WriteU32Le(&tx_packet.payload[84], telemetry->sample_count);
    WriteI32Le(&tx_packet.payload[88],
               FloatToScaledInt32(
                   telemetry->desired_yaw_rate_rad_s,
                   1000.0f));
    WriteI32Le(&tx_packet.payload[92],
               FloatToScaledInt32(
                   telemetry->yaw_rate_error_rad_s,
                   1000.0f));
    WriteI32Le(&tx_packet.payload[96],
               FloatToScaledInt32(
                   telemetry->yaw_rate_ff_correction_rad_s,
                   1000.0f));
    WriteI32Le(&tx_packet.payload[100],
               FloatToScaledInt32(
                   telemetry->yaw_rate_p_correction_rad_s,
                   1000.0f));
    WriteI32Le(&tx_packet.payload[104],
               FloatToScaledInt32(
                   telemetry->yaw_rate_i_correction_rad_s,
                   1000.0f));
    tx_packet.payload[108] = telemetry->heading_controller_version;
    tx_packet.payload[109] =
        telemetry->heading_correction_saturated;
    tx_packet.payload[110] = telemetry->motion_profile_status;
    tx_packet.payload[111] = telemetry->motion_profile_fault;
    size_t wire_length = 0U;
    if (amr_protocol_encode(&tx_packet,
                            tx_wire,
                            sizeof(tx_wire),
                            &wire_length) != AMR_PROTOCOL_OK)
    {
        return false;
    }
    return CDC_Transmit_FS(tx_wire, (uint16_t)wire_length) == USBD_OK;
}
static void ServiceOdometryTelemetry(void)
{
    TelemetryData_t snapshot;
    while (osMessageQueueGet(TelemetryQueueHandle,
                             &snapshot,
                             NULL,
                             0U) == osOK)
    {
        latest_telemetry = snapshot;
        have_telemetry_snapshot = true;
    }
    if (!odometry_stream_enabled || !have_telemetry_snapshot)
    {
        return;
    }
    const uint32_t now_ms = osKernelGetTickCount();
    if ((int32_t)(now_ms - next_odometry_tx_ms) < 0)
    {
        return;
    }
    uint32_t stream_slot_ms = odometry_stream_period_ms;
    if (drive_diagnostic_stream_enabled)
    {
        /*
         * Keep /odom alive while drive diagnostics are enabled.
         * With stream_period_ms = 50 ms:
         *   0 ms  -> ODOM
         *   25 ms -> DRIVE_DIAGNOSTIC
         *   50 ms -> ODOM
         *   75 ms -> DRIVE_DIAGNOSTIC
         * Each stream is therefore approximately 20 Hz.
         */
        if (stream_send_diagnostic_next)
        {
            (void)SendDriveDiagnosticTelemetry(&latest_telemetry);
        }
        else
        {
            (void)SendOdometryTelemetry(&latest_telemetry);
        }
        stream_send_diagnostic_next = !stream_send_diagnostic_next;
        stream_slot_ms =
            ((uint32_t)odometry_stream_period_ms) / 2U;
        if (stream_slot_ms < COMM_ODOM_MIN_PERIOD_MS)
        {
            stream_slot_ms = COMM_ODOM_MIN_PERIOD_MS;
        }
    }
    else
    {
        stream_send_diagnostic_next = false;
        (void)SendOdometryTelemetry(&latest_telemetry);
    }
    next_odometry_tx_ms = now_ms + stream_slot_ms;
}
static void HandleSetMode(const amr_packet_t *packet)
{
    if (!PacketTargetsDrive(packet))
    {
        return;
    }
    if (packet->payload_length != 1U)
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_FORMAT);
        return;
    }
    uint8_t requested_mode = packet->payload[0];
    if ((requested_mode != AMR_MODE_SAFE) &&
        (requested_mode != AMR_MODE_MANUAL) &&
        (requested_mode != AMR_MODE_AUTO))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_RANGE);
        return;
    }
    uint32_t flags = osEventFlagsGet(SystemEventsHandle);
    if ((requested_mode != AMR_MODE_SAFE) &&
        ((flags & (AMR_EVT_FAULT | AMR_EVT_ESTOP)) != 0U))
    {
        ReplyIfRequired(packet, false, COMM_DETAIL_STATE);
        return;
    }
    current_mode = requested_mode;
    if (requested_mode == AMR_MODE_SAFE)
    {
        StopMotionRequest();
        have_drive_sequence = false;
    }
    ReplyIfRequired(packet, true, COMM_DETAIL_NONE);
}
static void HandleEstop(const amr_packet_t *packet)
{
    if (!PacketTargetsDrive(packet))
    {
        return;
    }
    current_mode = AMR_MODE_ESTOP;
    StopMotionRequest();
    (void)osEventFlagsSet(
        SystemEventsHandle,
        AMR_EVT_ESTOP | AMR_EVT_FAULT);
    ReplyIfRequired(packet, true, COMM_DETAIL_NONE);
}
static void HandleClearFault(const amr_packet_t *packet)
{
    if (!PacketTargetsDrive(packet))
    {
        return;
    }
    current_mode = AMR_MODE_SAFE;
    StopMotionRequest();
    have_drive_sequence = false;
    (void)osEventFlagsClear(
        SystemEventsHandle,
        AMR_EVT_ESTOP | AMR_EVT_FAULT);
    ReplyIfRequired(packet, true, COMM_DETAIL_NONE);
}
static void ProcessPacket(const amr_packet_t *packet)
{
    last_host_activity_ms = osKernelGetTickCount();
    (void)osEventFlagsSet(
        SystemEventsHandle,
        AMR_EVT_USB_CONNECTED);
    switch (packet->message_id)
    {
        case AMR_MSG_HEARTBEAT:
        {
            amr_heartbeat_t heartbeat;
            if (!amr_heartbeat_decode(packet->payload,
                                      packet->payload_length,
                                      &heartbeat))
            {
                ReplyIfRequired(packet, false, COMM_DETAIL_FORMAT);
                return;
            }
            ReplyIfRequired(packet, true, COMM_DETAIL_NONE);
            break;
        }
        case AMR_MSG_SET_MODE:
            HandleSetMode(packet);
            break;
        case AMR_MSG_DRIVE_COMMAND:
            HandleDriveCommand(packet);
            break;
        case AMR_MSG_TURN_COMMAND:
            HandleTurnCommand(packet);
            break;
        case AMR_MSG_DISTANCE_COMMAND:
            HandleDistanceCommand(packet);
            break;
        case AMR_MSG_ODOMETRY_STREAM_CONTROL:
            HandleOdometryStreamControl(packet);
            break;
        case AMR_MSG_ESTOP:
            HandleEstop(packet);
            break;
        case AMR_MSG_CLEAR_FAULT:
            HandleClearFault(packet);
            break;
        case AMR_MSG_HELLO:
        case AMR_MSG_GET_STATUS:
            ReplyIfRequired(packet, true, COMM_DETAIL_NONE);
            break;
        default:
            ReplyIfRequired(packet, false, COMM_DETAIL_UNSUPPORTED);
            break;
    }
}
void AppComm_Run(void)
{
    UsbRxChunk_t chunk;
    amr_stream_decoder_init(&decoder);
    for (;;)
    {
        if (osMessageQueueGet(UsbRxQueueHandle,
                              &chunk,
                              NULL,
                              10U) == osOK)
        {
            if (chunk.length <= AMR_USB_RX_CHUNK_SIZE)
            {
                for (uint16_t index = 0U; index < chunk.length; ++index)
                {
                    const amr_protocol_result_t result =
                        amr_protocol_feed(&decoder,
                                          chunk.data[index],
                                          &rx_packet);
                    if (result == AMR_PROTOCOL_FRAME_READY)
                    {
                        ProcessPacket(&rx_packet);
                    }
                }
            }
        }
        ServiceOdometryTelemetry();
    }
}
uint32_t AppComm_GetLastHostActivityMs(void)
{
    return last_host_activity_ms;
}
uint8_t AppComm_GetMode(void)
{
    return current_mode;
}
/*
 * app_comm.c
 *
 *  Created on: Jul 29, 2026
 *      Author: Thai Vinh
 */
