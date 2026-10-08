#include "amr/amr_crc16.h"

uint16_t amr_crc16_ccitt_false(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    for (size_t index = 0U; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

