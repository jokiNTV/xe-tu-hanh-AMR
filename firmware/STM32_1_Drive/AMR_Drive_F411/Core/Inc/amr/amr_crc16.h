#ifndef AMR_CRC16_H
#define AMR_CRC16_H

#include <stddef.h>
#include <stdint.h>

uint16_t amr_crc16_ccitt_false(const uint8_t *data, size_t length);

#endif

