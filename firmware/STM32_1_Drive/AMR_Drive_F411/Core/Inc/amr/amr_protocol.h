#ifndef AMR_PROTOCOL_H
#define AMR_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AMR_PROTOCOL_VERSION 1U
#define AMR_MAGIC_0 0x41U
#define AMR_MAGIC_1 0x4DU
#define AMR_MAX_PAYLOAD_SIZE 192U
#define AMR_HEADER_SIZE 14U
#define AMR_CRC_SIZE 2U
#define AMR_MAX_RAW_FRAME_SIZE \
    (AMR_HEADER_SIZE + AMR_MAX_PAYLOAD_SIZE + AMR_CRC_SIZE)
#define AMR_MAX_ENCODED_FRAME_SIZE \
    (AMR_MAX_RAW_FRAME_SIZE + (AMR_MAX_RAW_FRAME_SIZE / 254U) + 1U)
#define AMR_MAX_WIRE_FRAME_SIZE (AMR_MAX_ENCODED_FRAME_SIZE + 1U)

typedef enum {
    AMR_DEVICE_HOST = 0x01,
    AMR_DEVICE_DRIVE = 0x10,
    AMR_DEVICE_MECHANISM = 0x11,
    AMR_DEVICE_BROADCAST = 0xFF
} amr_device_id_t;

typedef enum {
    AMR_MSG_HELLO = 0x01,
    AMR_MSG_HEARTBEAT = 0x02,
    AMR_MSG_ACK = 0x03,
    AMR_MSG_NACK = 0x04,
    AMR_MSG_GET_STATUS = 0x05,
    AMR_MSG_CLEAR_FAULT = 0x06,
    AMR_MSG_SET_MODE = 0x07,
    AMR_MSG_ESTOP = 0x08,
    AMR_MSG_DRIVE_COMMAND = 0x20,
    AMR_MSG_DRIVE_TELEMETRY = 0x21,
    AMR_MSG_IMU_TELEMETRY = 0x22,
    AMR_MSG_DRIVE_CONFIG = 0x23,
    AMR_MSG_TURN_COMMAND = 0x24,
    AMR_MSG_DISTANCE_COMMAND = 0x25,
    AMR_MSG_ODOMETRY_STREAM_CONTROL = 0x26,
    AMR_MSG_ODOMETRY = 0x27,
    AMR_MSG_DRIVE_DIAGNOSTIC = 0x28,
    AMR_MSG_MECHANISM_COMMAND = 0x40,
    AMR_MSG_MECHANISM_STATUS = 0x41,
    AMR_MSG_MECHANISM_CONFIG = 0x42
} amr_message_id_t;

typedef enum {
    AMR_FLAG_NONE = 0,
    AMR_FLAG_ACK_REQUIRED = 1U << 0,
    AMR_FLAG_IS_RESPONSE = 1U << 1,
    AMR_FLAG_ERROR = 1U << 2
} amr_packet_flags_t;

typedef struct {
    uint8_t device_id;
    uint8_t message_id;
    uint8_t flags;
    uint16_t sequence;
    uint32_t timestamp_ms;
    uint16_t payload_length;
    uint8_t payload[AMR_MAX_PAYLOAD_SIZE];
} amr_packet_t;

typedef enum {
    AMR_PROTOCOL_OK = 0,
    AMR_PROTOCOL_NO_FRAME,
    AMR_PROTOCOL_FRAME_READY,
    AMR_PROTOCOL_ERR_ARGUMENT,
    AMR_PROTOCOL_ERR_CAPACITY,
    AMR_PROTOCOL_ERR_COBS,
    AMR_PROTOCOL_ERR_LENGTH,
    AMR_PROTOCOL_ERR_MAGIC,
    AMR_PROTOCOL_ERR_VERSION,
    AMR_PROTOCOL_ERR_CRC,
    AMR_PROTOCOL_ERR_OVERFLOW
} amr_protocol_result_t;

typedef struct {
    uint8_t encoded_frame[AMR_MAX_ENCODED_FRAME_SIZE];
    size_t encoded_length;
    bool overflow;
} amr_stream_decoder_t;

void amr_stream_decoder_init(amr_stream_decoder_t *decoder);

amr_protocol_result_t amr_protocol_encode(const amr_packet_t *packet,
                                          uint8_t *wire_frame,
                                          size_t wire_capacity,
                                          size_t *wire_length);

amr_protocol_result_t amr_protocol_decode_frame(const uint8_t *encoded_frame,
                                                size_t encoded_length,
                                                amr_packet_t *packet);

amr_protocol_result_t amr_protocol_feed(amr_stream_decoder_t *decoder,
                                        uint8_t byte,
                                        amr_packet_t *packet);

#endif
