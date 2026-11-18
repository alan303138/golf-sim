#include "gs_cobs.h"

size_t gs_cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap)
{
    if (out_cap < GS_COBS_MAX_ENCODED(len)) {
        return 0;
    }
    size_t code_idx = 0; /* where the current block's code byte goes */
    size_t o = 1;
    uint8_t code = 1;
    for (size_t i = 0; i < len; i++) {
        if (in[i] == 0) {
            out[code_idx] = code;
            code_idx = o++;
            code = 1;
        } else {
            out[o++] = in[i];
            code++;
            if (code == 0xFF) {
                out[code_idx] = code;
                code_idx = o++;
                code = 1;
            }
        }
    }
    out[code_idx] = code;
    return o;
}

size_t gs_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap)
{
    size_t i = 0;
    size_t o = 0;
    while (i < len) {
        uint8_t code = in[i];
        if (code == 0) {
            return GS_COBS_ERROR;
        }
        i++;
        for (uint8_t k = 1; k < code; k++) {
            if (i >= len || in[i] == 0 || o >= out_cap) {
                return GS_COBS_ERROR;
            }
            out[o++] = in[i++];
        }
        /* A code < 0xFF implies a zero byte, except at the very end. */
        if (code != 0xFF && i < len) {
            if (o >= out_cap) {
                return GS_COBS_ERROR;
            }
            out[o++] = 0;
        }
    }
    return o;
}
