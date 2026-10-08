#ifndef AMR_MESSAGES_H
#define AMR_MESSAGES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    AMR_MODE_SAFE = 0,
    AMR_MODE_MANUAL = 1,
    AMR_MODE_AUTO = 2,
    AMR_MODE_FAULT = 3,
    AMR_MODE_ESTOP = 4
} amr_mode_t;

typedef enum {
    AMR_MECH_CMD_NONE = 0,
    AMR_MECH_CMD_HOME = 1,
    AMR_MECH_CMD_OPEN = 2,
    AMR_MECH_CMD_GRIP = 3,
    AMR_MECH_CMD_LIFT = 4,
    AMR_MECH_CMD_LOWER = 5,
    AMR_MECH_CMD_RELEASE = 6,
    AMR_MECH_CMD_STOP = 7
} amr_mechanism_command_id_t;

typedef struct {
    uint32_t uptime_ms;
    uint32_t health_bits;
} amr_heartbeat_t;

typedef struct {
    uint16_t acknowledged_sequence;
    uint8_t status;
    uint8_t detail;
} amr_ack_t;

typedef struct {
    int32_t left_target_mrad_s;
    int32_t right_target_mrad_s;
    uint16_t command_timeout_ms;
    uint16_t reserved;
} amr_drive_command_t;

typedef struct {
    int32_t relative_angle_mrad;
    uint16_t max_yaw_rate_mrad_s;
    uint16_t command_timeout_ms;
    uint32_t command_token;
} amr_turn_command_t;

typedef struct {
    int32_t distance_mm;
    uint16_t max_speed_mm_s;
    uint16_t command_timeout_ms;
    uint32_t command_token;
} amr_distance_command_t;

typedef struct {
    uint16_t period_ms;
    uint16_t options;
} amr_odometry_stream_control_t;

enum {
    AMR_ODOMETRY_STATUS_VALID = 1U << 0
};

typedef struct {
    uint32_t sample_timestamp_ms;
    int32_t x_mm;
    int32_t y_mm;
    int32_t yaw_mrad;
    int32_t linear_velocity_mm_s;
    int32_t angular_velocity_mrad_s;
    uint32_t sample_count;
    uint8_t status;
    uint8_t reserved[3];
} amr_odometry_t;

typedef struct {
    int32_t left_target_mrad_s;
    int32_t right_target_mrad_s;
    int32_t left_measured_mrad_s;
    int32_t right_measured_mrad_s;
    int64_t left_encoder_count;
    int64_t right_encoder_count;
    int16_t left_output_permille;
    int16_t right_output_permille;
    uint8_t mode;
    uint8_t fault;
    uint16_t command_age_ms;
} amr_drive_telemetry_t;

typedef struct {
    uint8_t command;
    uint8_t options;
    uint16_t command_timeout_ms;
    uint32_t command_token;
} amr_mechanism_command_t;

typedef struct {
    uint8_t state;
    uint8_t active_command;
    uint16_t fault;
    uint16_t sensor_flags;
    uint16_t reserved;
    int32_t lift_left_current_ma;
    int32_t lift_right_current_ma;
    int32_t gripper_current_ma;
    uint32_t command_token;
} amr_mechanism_status_t;

enum {
    AMR_HEARTBEAT_PAYLOAD_SIZE = 8,
    AMR_ACK_PAYLOAD_SIZE = 4,
    AMR_DRIVE_COMMAND_PAYLOAD_SIZE = 12,
    AMR_TURN_COMMAND_PAYLOAD_SIZE = 12,
    AMR_DISTANCE_COMMAND_PAYLOAD_SIZE = 12,
    AMR_ODOMETRY_STREAM_CONTROL_PAYLOAD_SIZE = 4,
    AMR_ODOMETRY_PAYLOAD_SIZE = 32,
    AMR_DRIVE_TELEMETRY_PAYLOAD_SIZE = 40,
    AMR_MECHANISM_COMMAND_PAYLOAD_SIZE = 8,
    AMR_MECHANISM_STATUS_PAYLOAD_SIZE = 24
};

bool amr_heartbeat_encode(const amr_heartbeat_t *message,
                          uint8_t *payload,
                          size_t capacity);
bool amr_heartbeat_decode(const uint8_t *payload,
                          size_t length,
                          amr_heartbeat_t *message);

bool amr_ack_encode(const amr_ack_t *message,
                    uint8_t *payload,
                    size_t capacity);
bool amr_ack_decode(const uint8_t *payload,
                    size_t length,
                    amr_ack_t *message);

bool amr_drive_command_encode(const amr_drive_command_t *message,
                              uint8_t *payload,
                              size_t capacity);
bool amr_drive_command_decode(const uint8_t *payload,
                              size_t length,
                              amr_drive_command_t *message);

bool amr_turn_command_encode(const amr_turn_command_t *message,
                             uint8_t *payload,
                             size_t capacity);
bool amr_turn_command_decode(const uint8_t *payload,
                             size_t length,
                             amr_turn_command_t *message);

bool amr_distance_command_encode(const amr_distance_command_t *message,
                                 uint8_t *payload,
                                 size_t capacity);
bool amr_distance_command_decode(const uint8_t *payload,
                                 size_t length,
                                 amr_distance_command_t *message);

bool amr_odometry_stream_control_encode(
    const amr_odometry_stream_control_t *message,
    uint8_t *payload,
    size_t capacity);
bool amr_odometry_stream_control_decode(
    const uint8_t *payload,
    size_t length,
    amr_odometry_stream_control_t *message);

bool amr_odometry_encode(const amr_odometry_t *message,
                         uint8_t *payload,
                         size_t capacity);
bool amr_odometry_decode(const uint8_t *payload,
                         size_t length,
                         amr_odometry_t *message);

bool amr_drive_telemetry_encode(const amr_drive_telemetry_t *message,
                                uint8_t *payload,
                                size_t capacity);
bool amr_drive_telemetry_decode(const uint8_t *payload,
                                size_t length,
                                amr_drive_telemetry_t *message);

bool amr_mechanism_command_encode(const amr_mechanism_command_t *message,
                                  uint8_t *payload,
                                  size_t capacity);
bool amr_mechanism_command_decode(const uint8_t *payload,
                                  size_t length,
                                  amr_mechanism_command_t *message);

bool amr_mechanism_status_encode(const amr_mechanism_status_t *message,
                                 uint8_t *payload,
                                 size_t capacity);
bool amr_mechanism_status_decode(const uint8_t *payload,
                                 size_t length,
                                 amr_mechanism_status_t *message);

#endif
