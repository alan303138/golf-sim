#include "gs_crc16.h"

/* Nibble-table implementation: small (32 bytes of table) and fast enough on a
 * Cortex-M0+ for the link rates involved (< 1 MB/s). */
static const uint16_t k_crc_nibble[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
};

uint16_t gs_crc16_update(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        crc = (uint16_t)((crc << 4) ^ k_crc_nibble[((crc >> 12) ^ (b >> 4)) & 0x0F]);
        crc = (uint16_t)((crc << 4) ^ k_crc_nibble[((crc >> 12) ^ (b & 0x0F)) & 0x0F]);
    }
    return crc;
}

uint16_t gs_crc16(const uint8_t *data, size_t len)
{
    return gs_crc16_update(GS_CRC16_INIT, data, len);
}
