/*
 * Consistent Overhead Byte Stuffing (COBS), standard variant
 * (Cheshire & Baker 1999). The encoded output never contains 0x00, so 0x00 is
 * used as the frame delimiter on the wire. The delimiter is NOT written by
 * these functions.
 * Portable C, no pico-sdk dependency.
 */
#ifndef GS_COBS_H
#define GS_COBS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Worst-case encoded size for len input bytes (excluding delimiter). */
#define GS_COBS_MAX_ENCODED(len) ((len) + ((len) / 254u) + 1u)

/* Encode len bytes from in into out (capacity out_cap).
 * Returns encoded length, or 0 if out_cap is too small. */
size_t gs_cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);

/* Decode len bytes (one frame, without the 0x00 delimiter) into out.
 * Returns decoded length, or (size_t)-1 on malformed input / overflow.
 * in and out may be the same buffer (in-place decode is safe). */
size_t gs_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);

#define GS_COBS_ERROR ((size_t)-1)

#ifdef __cplusplus
}
#endif

#endif
