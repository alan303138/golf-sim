/*
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0).
 * Check value: crc("123456789") == 0x29B1.
 * Portable C, no pico-sdk dependency.
 */
#ifndef GS_CRC16_H
#define GS_CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GS_CRC16_INIT 0xFFFFu

/* Continue a CRC over more data. Start with crc = GS_CRC16_INIT. */
uint16_t gs_crc16_update(uint16_t crc, const uint8_t *data, size_t len);

/* One-shot CRC over a buffer. */
uint16_t gs_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
