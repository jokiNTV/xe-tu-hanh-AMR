#include "amr/amr_messages.h"

static void write_u16_le(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void write_u32_le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8U) & 0xFFU);
    output[2] = (uint8_t)((value >> 16U) & 0xFFU);
    output[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void write_u64_le(uint8_t *output, uint64_t value)
{
    for (uint8_t index = 0U; index < 8U; ++index) {
        output[index] = (uint8_t)((value >> (8U * index)) & 0xFFU);
    }
}

static uint16_t read_u16_le(const uint8_t *input)
{
    return (uint16_t)input[0] | ((uint16_t)input[1] << 8U);
}

static uint32_t read_u32_le(const uint8_t *input)
{
    return (uint32_t)input[0] |
           ((uint32_t)input[1] << 8U) |
           ((uint32_t)input[2] << 16U) |
           ((uint32_t)input[3] << 24U);
}

static uint64_t read_u64_le(const uint8_t *input)
{
    uint64_t value = 0U;
    for (uint8_t index = 0U; index < 8U; ++index) {
        value |= (uint64_t)input[index] << (8U * index);
    }
    return value;
}

bool amr_heartbeat_encode(const amr_heartbeat_t *message,
                          uint8_t *payload,
                          size_t capacity)
{
    if (message == NULL || payload == NULL ||
        capacity < AMR_HEARTBEAT_PAYLOAD_SIZE) {
        return false;
    }
    write_u32_le(&payload[0], message->uptime_ms);
    write_u32_le(&payload[4], message->health_bits);
    return true;
}

bool amr_heartbeat_decode(const uint8_t *payload,
                          size_t length,
                          amr_heartbeat_t *message)
{
    if (payload == NULL || message == NULL ||
        length != AMR_HEARTBEAT_PAYLOAD_SIZE) {
        return false;
    }
    message->uptime_ms = read_u32_le(&payload[0]);
    message->health_bits = read_u32_le(&payload[4]);
    return true;
}

bool amr_ack_encode(const amr_ack_t *message,
                    uint8_t *payload,
                    size_t capacity)
{
    if (message == NULL || payload == NULL || capacity < AMR_ACK_PAYLOAD_SIZE) {
        return false;
    }
    write_u16_le(&payload[0], message->acknowledged_sequence);
    payload[2] = message->status;
    payload[3] = message->detail;
    return true;
}

bool amr_ack_decode(const uint8_t *payload,
                    size_t length,
                    amr_ack_t *message)
{
    if (payload == NULL || message == NULL || length != AMR_ACK_PAYLOAD_SIZE) {
        return false;
    }
    message->acknowledged_sequence = read_u16_le(&payload[0]);
    message->status = payload[2];
    message->detail = payload[3];
    return true;
}

bool amr_drive_command_encode(const amr_drive_command_t *message,
                              uint8_t *payload,
                              size_t capacity)
{
    if (message == NULL || payload == NULL ||
        capacity < AMR_DRIVE_COMMAND_PAYLOAD_SIZE) {
        return false;
    }
    write_u32_le(&payload[0], (uint32_t)message->left_target_mrad_s);
    write_u32_le(&payload[4], (uint32_t)message->right_target_mrad_s);
    write_u16_le(&payload[8], message->command_timeout_ms);
    write_u16_le(&payload[10], message->reserved);
    return true;
}

bool amr_drive_command_decode(const uint8_t *payload,
                              size_t length,
                              amr_drive_command_t *message)
{
    if (payload == NULL || message == NULL ||
        length != AMR_DRIVE_COMMAND_PAYLOAD_SIZE) {
        return false;
    }
    message->left_target_mrad_s = (int32_t)read_u32_le(&payload[0]);
    message->right_target_mrad_s = (int32_t)read_u32_le(&payload[4]);
    message->command_timeout_ms = read_u16_le(&payload[8]);
    message->reserved = read_u16_le(&payload[10]);
    return true;
}

bool amr_turn_command_encode(const amr_turn_command_t *message,
                             uint8_t *payload,
                             size_t capacity)
{
    if (message == NULL || payload == NULL ||
        capacity < AMR_TURN_COMMAND_PAYLOAD_SIZE) {
        return false;
    }
    write_u32_le(&payload[0], (uint32_t)message->relative_angle_mrad);
    write_u16_le(&payload[4], message->max_yaw_rate_mrad_s);
    write_u16_le(&payload[6], message->command_timeout_ms);
    write_u32_le(&payload[8], message->command_token);
    return true;
}

bool amr_turn_command_decode(const uint8_t *payload,
                             size_t length,
                             amr_turn_command_t *message)
{
    if (payload == NULL || message == NULL ||
        length != AMR_TURN_COMMAND_PAYLOAD_SIZE) {
        return false;
    }
    message->relative_angle_mrad = (int32_t)read_u32_le(&payload[0]);
    message->max_yaw_rate_mrad_s = read_u16_le(&payload[4]);
    message->command_timeout_ms = read_u16_le(&payload[6]);
    message->command_token = read_u32_le(&payload[8]);
    return true;
}

bool amr_distance_command_encode(const amr_distance_command_t *message,
                                 uint8_t *payload,
                                 size_t capacity)
{
    if (message == NULL || payload == NULL ||
        capacity < AMR_DISTANCE_COMMAND_PAYLOAD_SIZE) {
        return false;
    }
    write_u32_le(&payload[0], (uint32_t)message->distance_mm);
    write_u16_le(&payload[4], message->max_speed_mm_s);
    write_u16_le(&payload[6], message->command_timeout_ms);
    write_u32_le(&payload[8], message->command_token);
    return true;
}

bool amr_distance_command_decode(const uint8_t *payload,
                                 size_t length,
                                 amr_distance_command_t *message)
{
    if (payload == NULL || message == NULL ||
        length != AMR_DISTANCE_COMMAND_PAYLOAD_SIZE) {
        return false;
    }
    message->distance_mm = (int32_t)read_u32_le(&payload[0]);
    message->max_speed_mm_s = read_u16_le(&payload[4]);
    message->command_timeout_ms = read_u16_le(&payload[6]);
    message->command_token = read_u32_le(&payload[8]);
    return true;
}

bool amr_odometry_stream_control_encode(
    const amr_odometry_stream_control_t *message,
    uint8_t *payload,
    size_t capacity)
{
    if ((message == NULL) || (payload == NULL) ||
        (capacity < AMR_ODOMETRY_STREAM_CONTROL_PAYLOAD_SIZE)) {
        return false;
    }

    write_u16_le(&payload[0], message->period_ms);
    write_u16_le(&payload[2], message->options);
    return true;
}

bool amr_odometry_stream_control_decode(
    const uint8_t *payload,
    size_t length,
    amr_odometry_stream_control_t *message)
{
    if ((payload == NULL) || (message == NULL) ||
        (length != AMR_ODOMETRY_STREAM_CONTROL_PAYLOAD_SIZE)) {
        return false;
    }

    message->period_ms = read_u16_le(&payload[0]);
    message->options = read_u16_le(&payload[2]);
    return true;
}

bool amr_odometry_encode(const amr_odometry_t *message,
                         uint8_t *payload,
                         size_t capacity)
{
    if ((message == NULL) || (payload == NULL) ||
        (capacity < AMR_ODOMETRY_PAYLOAD_SIZE)) {
        return false;
    }

    write_u32_le(&payload[0], message->sample_timestamp_ms);
    write_u32_le(&payload[4], (uint32_t)message->x_mm);
    write_u32_le(&payload[8], (uint32_t)message->y_mm);
    write_u32_le(&payload[12], (uint32_t)message->yaw_mrad);
    write_u32_le(&payload[16],
                 (uint32_t)message->linear_velocity_mm_s);
    write_u32_le(&payload[20],
                 (uint32_t)message->angular_velocity_mrad_s);
    write_u32_le(&payload[24], message->sample_count);
    payload[28] = message->status;
    payload[29] = message->reserved[0];
    payload[30] = message->reserved[1];
    payload[31] = message->reserved[2];
    return true;
}

bool amr_odometry_decode(const uint8_t *payload,
                         size_t length,
                         amr_odometry_t *message)
{
    if ((payload == NULL) || (message == NULL) ||
        (length != AMR_ODOMETRY_PAYLOAD_SIZE)) {
        return false;
    }

    message->sample_timestamp_ms = read_u32_le(&payload[0]);
    message->x_mm = (int32_t)read_u32_le(&payload[4]);
    message->y_mm = (int32_t)read_u32_le(&payload[8]);
    message->yaw_mrad = (int32_t)read_u32_le(&payload[12]);
    message->linear_velocity_mm_s = (int32_t)read_u32_le(&payload[16]);
    message->angular_velocity_mrad_s = (int32_t)read_u32_le(&payload[20]);
    message->sample_count = read_u32_le(&payload[24]);
    message->status = payload[28];
    message->reserved[0] = payload[29];
    message->reserved[1] = payload[30];
    message->reserved[2] = payload[31];
    return true;
}

bool amr_drive_telemetry_encode(const amr_drive_telemetry_t *message,
                                uint8_t *payload,
                                size_t capacity)
{
    if (message == NULL || payload == NULL ||
        capacity < AMR_DRIVE_TELEMETRY_PAYLOAD_SIZE) {
        return false;
    }
    write_u32_le(&payload[0], (uint32_t)message->left_target_mrad_s);
    write_u32_le(&payload[4], (uint32_t)message->right_target_mrad_s);
    write_u32_le(&payload[8], (uint32_t)message->left_measured_mrad_s);
    write_u32_le(&payload[12], (uint32_t)message->right_measured_mrad_s);
    write_u64_le(&payload[16], (uint64_t)message->left_encoder_count);
    write_u64_le(&payload[24], (uint64_t)message->right_encoder_count);
    write_u16_le(&payload[32], (uint16_t)message->left_output_permille);
    write_u16_le(&payload[34], (uint16_t)message->right_output_permille);
    payload[36] = message->mode;
    payload[37] = message->fault;
    write_u16_le(&payload[38], message->command_age_ms);
    return true;
}

bool amr_drive_telemetry_decode(const uint8_t *payload,
                                size_t length,
                                amr_drive_telemetry_t *message)
{
    if (payload == NULL || message == NULL ||
        length != AMR_DRIVE_TELEMETRY_PAYLOAD_SIZE) {
        return false;
    }
    message->left_target_mrad_s = (int32_t)read_u32_le(&payload[0]);
    message->right_target_mrad_s = (int32_t)read_u32_le(&payload[4]);
    message->left_measured_mrad_s = (int32_t)read_u32_le(&payload[8]);
    message->right_measured_mrad_s = (int32_t)read_u32_le(&payload[12]);
    message->left_encoder_count = (int64_t)read_u64_le(&payload[16]);
    message->right_encoder_count = (int64_t)read_u64_le(&payload[24]);
    message->left_output_permille = (int16_t)read_u16_le(&payload[32]);
    message->right_output_permille = (int16_t)read_u16_le(&payload[34]);
    message->mode = payload[36];
    message->fault = payload[37];
    message->command_age_ms = read_u16_le(&payload[38]);
    return true;
}

bool amr_mechanism_command_encode(const amr_mechanism_command_t *message,
                                  uint8_t *payload,
                                  size_t capacity)
{
    if (message == NULL || payload == NULL ||
        capacity < AMR_MECHANISM_COMMAND_PAYLOAD_SIZE) {
        return false;
    }
    payload[0] = message->command;
    payload[1] = message->options;
    write_u16_le(&payload[2], message->command_timeout_ms);
    write_u32_le(&payload[4], message->command_token);
    return true;
}

bool amr_mechanism_command_decode(const uint8_t *payload,
                                  size_t length,
                                  amr_mechanism_command_t *message)
{
    if (payload == NULL || message == NULL ||
        length != AMR_MECHANISM_COMMAND_PAYLOAD_SIZE) {
        return false;
    }
    message->command = payload[0];
    message->options = payload[1];
    message->command_timeout_ms = read_u16_le(&payload[2]);
    message->command_token = read_u32_le(&payload[4]);
    return true;
}

bool amr_mechanism_status_encode(const amr_mechanism_status_t *message,
                                 uint8_t *payload,
                                 size_t capacity)
{
    if (message == NULL || payload == NULL ||
        capacity < AMR_MECHANISM_STATUS_PAYLOAD_SIZE) {
        return false;
    }
    payload[0] = message->state;
    payload[1] = message->active_command;
    write_u16_le(&payload[2], message->fault);
    write_u16_le(&payload[4], message->sensor_flags);
    write_u16_le(&payload[6], message->reserved);
    write_u32_le(&payload[8], (uint32_t)message->lift_left_current_ma);
    write_u32_le(&payload[12], (uint32_t)message->lift_right_current_ma);
    write_u32_le(&payload[16], (uint32_t)message->gripper_current_ma);
    write_u32_le(&payload[20], message->command_token);
    return true;
}

bool amr_mechanism_status_decode(const uint8_t *payload,
                                 size_t length,
                                 amr_mechanism_status_t *message)
{
    if (payload == NULL || message == NULL ||
        length != AMR_MECHANISM_STATUS_PAYLOAD_SIZE) {
        return false;
    }
    message->state = payload[0];
    message->active_command = payload[1];
    message->fault = read_u16_le(&payload[2]);
    message->sensor_flags = read_u16_le(&payload[4]);
    message->reserved = read_u16_le(&payload[6]);
    message->lift_left_current_ma = (int32_t)read_u32_le(&payload[8]);
    message->lift_right_current_ma = (int32_t)read_u32_le(&payload[12]);
    message->gripper_current_ma = (int32_t)read_u32_le(&payload[16]);
    message->command_token = read_u32_le(&payload[20]);
    return true;
}
