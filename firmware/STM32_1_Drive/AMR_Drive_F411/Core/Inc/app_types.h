#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>

#define AMR_USB_RX_CHUNK_SIZE          64U
#define AMR_CMD_TIMEOUT_DEFAULT_MS    250U

/* SystemEvents flags */
#define AMR_EVT_USB_CONNECTED     (1UL << 0)
#define AMR_EVT_CMD_VALID         (1UL << 1)
#define AMR_EVT_IMU_READY         (1UL << 2)
#define AMR_EVT_MOTOR_ENABLED     (1UL << 3)
#define AMR_EVT_FAULT             (1UL << 4)
#define AMR_EVT_ESTOP             (1UL << 5)

typedef enum
{
    AMR_FAULT_NONE          = 0,
    AMR_FAULT_CMD_TIMEOUT   = (1UL << 0),
    AMR_FAULT_IMU           = (1UL << 1),
    AMR_FAULT_ENCODER_LEFT  = (1UL << 2),
    AMR_FAULT_ENCODER_RIGHT = (1UL << 3),
    AMR_FAULT_USB           = (1UL << 4)
} AmrFaultFlags_t;

typedef enum
{
    AMR_MOTION_COMMAND_WHEEL_SPEED = 0,
    AMR_MOTION_COMMAND_TURN_RELATIVE = 1,
    AMR_MOTION_COMMAND_DRIVE_DISTANCE = 2
} AmrMotionCommandType_t;

typedef struct
{
    uint8_t command_type;
    uint8_t reserved8[3];

    int32_t left_target_mrad_s;
    int32_t right_target_mrad_s;

    int32_t turn_angle_mrad;
    uint16_t turn_max_yaw_rate_mrad_s;
    uint16_t reserved16;

    int32_t distance_mm;
    uint16_t distance_max_speed_mm_s;
    uint16_t distance_reserved16;

    uint32_t received_at_ms;
    uint16_t sequence;
    uint16_t timeout_ms;
    uint32_t command_token;
} DriveCommand_t;

typedef struct
{
    uint16_t length;
    uint16_t reserved;
    uint8_t data[AMR_USB_RX_CHUNK_SIZE];
} UsbRxChunk_t;

typedef struct
{
    uint32_t timestamp_ms;
    float target_left_rad_s;
    float target_right_rad_s;
    float measured_left_rad_s;
    float measured_right_rad_s;
    int16_t pwm_left;
    int16_t pwm_right;
    float imu_yaw_rad;
    float gyro_z_rad_s;
    float heading_reference_rad;
    float heading_error_rad;
    float heading_correction_rad_s;
    float desired_yaw_rate_rad_s;
    float yaw_rate_error_rad_s;
    float yaw_rate_ff_correction_rad_s;
    float yaw_rate_p_correction_rad_s;
    float yaw_rate_i_correction_rad_s;
    float encoder_yaw_rate_rad_s;
    float yaw_rate_residual_rad_s;
    float accel_x_m_s2;
    float accel_y_m_s2;
    float accel_z_m_s2;
    float odom_x_m;
    float odom_y_m;
    float odom_yaw_rad;
    float linear_velocity_m_s;
    float angular_velocity_rad_s;
    uint32_t sample_count;
    uint32_t command_refresh_count;
    uint16_t command_age_ms;
    uint8_t command_active;
    uint8_t heading_hold_active;
    uint8_t imu_ready;
    uint8_t odom_valid;
    uint8_t mode;
    uint8_t heading_controller_version;
    uint8_t heading_correction_saturated;
    uint8_t motion_profile_status;
    uint8_t motion_profile_fault;
} TelemetryData_t;

#endif
