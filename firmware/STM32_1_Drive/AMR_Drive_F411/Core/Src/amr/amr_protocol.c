#include "amr/amr_protocol.h"

#include <string.h>

#include "amr/amr_cobs.h"
#include "amr/amr_crc16.h"

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

void amr_stream_decoder_init(amr_stream_decoder_t *decoder)
{
    if (decoder == NULL) {
        return;
    }
    decoder->encoded_length = 0U;
    decoder->overflow = false;
}

amr_protocol_result_t amr_protocol_encode(const amr_packet_t *packet,
                                          uint8_t *wire_frame,
                                          size_t wire_capacity,
                                          size_t *wire_length)
{
    if (packet == NULL || wire_frame == NULL || wire_length == NULL) {
        return AMR_PROTOCOL_ERR_ARGUMENT;
    }
    if (packet->payload_length > AMR_MAX_PAYLOAD_SIZE) {
        return AMR_PROTOCOL_ERR_LENGTH;
    }

    uint8_t raw_frame[AMR_MAX_RAW_FRAME_SIZE];
    raw_frame[0] = AMR_MAGIC_0;
    raw_frame[1] = AMR_MAGIC_1;
    raw_frame[2] = AMR_PROTOCOL_VERSION;
    raw_frame[3] = packet->device_id;
    raw_frame[4] = packet->message_id;
    raw_frame[5] = packet->flags;
    write_u16_le(&raw_frame[6], packet->sequence);
    write_u16_le(&raw_frame[8], packet->payload_length);
    write_u32_le(&raw_frame[10], packet->timestamp_ms);

    if (packet->payload_length != 0U) {
        memcpy(&raw_frame[AMR_HEADER_SIZE],
               packet->payload,
               packet->payload_length);
    }

    const size_t content_length = AMR_HEADER_SIZE + packet->payload_length;
    const uint16_t crc = amr_crc16_ccitt_false(raw_frame, content_length);
    write_u16_le(&raw_frame[content_length], crc);
    const size_t raw_length = content_length + AMR_CRC_SIZE;

    size_t encoded_length = 0U;
    if (wire_capacity < 2U ||
        !amr_cobs_encode(raw_frame,
                         raw_length,
                         wire_frame,
                         wire_capacity - 1U,
                         &encoded_length)) {
        return AMR_PROTOCOL_ERR_CAPACITY;
    }
    if (encoded_length >= wire_capacity) {
        return AMR_PROTOCOL_ERR_CAPACITY;
    }

    wire_frame[encoded_length] = 0U;
    *wire_length = encoded_length + 1U;
    return AMR_PROTOCOL_OK;
}

amr_protocol_result_t amr_protocol_decode_frame(const uint8_t *encoded_frame,
                                                size_t encoded_length,
                                                amr_packet_t *packet)
{
    if (encoded_frame == NULL || packet == NULL) {
        return AMR_PROTOCOL_ERR_ARGUMENT;
    }
    if (encoded_length == 0U || encoded_length > AMR_MAX_ENCODED_FRAME_SIZE) {
        return AMR_PROTOCOL_ERR_LENGTH;
    }

    uint8_t raw_frame[AMR_MAX_RAW_FRAME_SIZE];
    size_t raw_length = 0U;
    if (!amr_cobs_decode(encoded_frame,
                         encoded_length,
                         raw_frame,
                         sizeof(raw_frame),
                         &raw_length)) {
        return AMR_PROTOCOL_ERR_COBS;
    }

    if (raw_length < AMR_HEADER_SIZE + AMR_CRC_SIZE) {
        return AMR_PROTOCOL_ERR_LENGTH;
    }
    if (raw_frame[0] != AMR_MAGIC_0 || raw_frame[1] != AMR_MAGIC_1) {
        return AMR_PROTOCOL_ERR_MAGIC;
    }
    if (raw_frame[2] != AMR_PROTOCOL_VERSION) {
        return AMR_PROTOCOL_ERR_VERSION;
    }

    const uint16_t payload_length = read_u16_le(&raw_frame[8]);
    if (payload_length > AMR_MAX_PAYLOAD_SIZE ||
        raw_length != AMR_HEADER_SIZE + payload_length + AMR_CRC_SIZE) {
        return AMR_PROTOCOL_ERR_LENGTH;
    }

    const size_t content_length = AMR_HEADER_SIZE + payload_length;
    const uint16_t expected_crc = read_u16_le(&raw_frame[content_length]);
    const uint16_t actual_crc =
        amr_crc16_ccitt_false(raw_frame, content_length);
    if (expected_crc != actual_crc) {
        return AMR_PROTOCOL_ERR_CRC;
    }

    packet->device_id = raw_frame[3];
    packet->message_id = raw_frame[4];
    packet->flags = raw_frame[5];
    packet->sequence = read_u16_le(&raw_frame[6]);
    packet->payload_length = payload_length;
    packet->timestamp_ms = read_u32_le(&raw_frame[10]);
    if (payload_length != 0U) {
        memcpy(packet->payload, &raw_frame[AMR_HEADER_SIZE], payload_length);
    }

    return AMR_PROTOCOL_OK;
}

amr_protocol_result_t amr_protocol_feed(amr_stream_decoder_t *decoder,
                                        uint8_t byte,
                                        amr_packet_t *packet)
{
    if (decoder == NULL || packet == NULL) {
        return AMR_PROTOCOL_ERR_ARGUMENT;
    }

    if (byte != 0U) {
        if (!decoder->overflow) {
            if (decoder->encoded_length < AMR_MAX_ENCODED_FRAME_SIZE) {
                decoder->encoded_frame[decoder->encoded_length] = byte;
                ++decoder->encoded_length;
            } else {
                decoder->overflow = true;
            }
        }
        return AMR_PROTOCOL_NO_FRAME;
    }

    if (decoder->overflow) {
        amr_stream_decoder_init(decoder);
        return AMR_PROTOCOL_ERR_OVERFLOW;
    }
    if (decoder->encoded_length == 0U) {
        return AMR_PROTOCOL_NO_FRAME;
    }

    const amr_protocol_result_t result =
        amr_protocol_decode_frame(decoder->encoded_frame,
                                  decoder->encoded_length,
                                  packet);
    amr_stream_decoder_init(decoder);

    if (result == AMR_PROTOCOL_OK) {
        return AMR_PROTOCOL_FRAME_READY;
    }
    return result;
}

