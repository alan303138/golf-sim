/*
 * Wire protocol: framing (COBS + CRC-16), message types and body layouts.
 * Portable C, no pico-sdk dependency. See firmware/PROTOCOL.md.
 *
 * Frame on the wire:   COBS( type | tag | body... | crc16_le ) 0x00
 * crc16 = CRC-16/CCITT-FALSE over type|tag|body, transmitted little-endian.
 * All multi-byte integers in bodies are little-endian.
 */
#ifndef GS_PROTO_H
#define GS_PROTO_H

#include <stddef.h>
#include <stdint.h>

#include "gs_cobs.h"
#include "gs_crc16.h"
#include "gs_err.h"
#include "gs_sched.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GS_PROTO_MAX_BODY  200u
#define GS_PROTO_MAX_RAW   (2u + GS_PROTO_MAX_BODY + 2u)
#define GS_PROTO_MAX_FRAME (GS_COBS_MAX_ENCODED(GS_PROTO_MAX_RAW) + 1u)
#define GS_BOOTLOADER_MAGIC 0xB00710ADu

enum gs_msg_type {
    /* host -> MCU */
    GS_MSG_GET_INFO = 0x01,
    GS_MSG_SET_CONFIG = 0x02,
    GS_MSG_SET_OPTIONS = 0x03,
    GS_MSG_ARM = 0x04,
    GS_MSG_FIRE = 0x05,
    GS_MSG_GET_STATUS = 0x06,
    GS_MSG_GET_CONFIG = 0x07,
    GS_MSG_BOOTLOADER = 0x08,
    GS_MSG_CLEAR_FAULTS = 0x09,
    GS_MSG_GET_OPTIONS = 0x0A,
    /* MCU -> host */
    GS_MSG_RESULT = 0x80,
    GS_MSG_INFO = 0x81,
    GS_MSG_CONFIG = 0x82,
    GS_MSG_STATUS = 0x83,
    GS_MSG_TRIGGER = 0x84,
    GS_MSG_EVENT = 0x85,
    GS_MSG_OPTIONS = 0x86
};

typedef struct {
    uint8_t type;
    uint8_t tag;
    uint16_t body_len;
    uint8_t body[GS_PROTO_MAX_BODY];
} gs_frame_t;

/* Encode a frame including the trailing 0x00 delimiter. Returns bytes written,
 * 0 if body too long or out_cap too small (needs GS_PROTO_MAX_FRAME). */
size_t gs_frame_encode(uint8_t type, uint8_t tag, const uint8_t *body, size_t body_len,
                       uint8_t *out, size_t out_cap);

/* Incremental deframer. Feed bytes; returns 1 when *out holds a valid frame.
 * Frames with bad COBS, bad CRC, bad length or overflow are dropped and
 * counted in bad_frames. */
typedef struct {
    uint8_t buf[GS_PROTO_MAX_FRAME];
    size_t len;
    int overflow;
    uint32_t bad_frames;
} gs_deframer_t;

void gs_deframer_init(gs_deframer_t *d);
int gs_deframer_push(gs_deframer_t *d, uint8_t byte, gs_frame_t *out);

/* ---- message bodies ---------------------------------------------------- */
typedef struct {
    uint8_t proto_major, proto_minor, fw_major, fw_minor, fw_patch, hw_revision;
    uint8_t uid[8];
    uint32_t uptime_ms, clk_hz;
    uint32_t max_width_us, min_width_us, max_duty_ppm, duty_window_us, max_pps;
    uint8_t max_pulses_per_exposure, max_burst_exposures;
    uint32_t max_exposure_us;
} gs_info_t;
#define GS_INFO_LEN 48u

typedef struct {
    uint8_t armed, mode, interlock_ok, flags;     /* flags bit0: strobe forced low */
    uint32_t faults, uptime_ms, rail_mv;
    uint32_t trig_hw, trig_sw, trig_inhibited, bursts_done, pulses_total, rx_bad_frames;
    uint32_t duty_ppm_now, max_pulse_us_seen;
} gs_status_t;
#define GS_STATUS_LEN 48u
#define GS_STATUS_FLAG_STROBE_FORCED_LOW 0x01u

#define GS_TRIG_SRC_HARDWARE 0u
#define GS_TRIG_SRC_SOFTWARE 1u
#define GS_TRIG_FLAG_INHIBITED 0x01u   /* strobe suppressed (interlock open); n_pulses = 0 */
#define GS_TRIG_FLAG_FREE_RUN  0x02u   /* free_run: exposure deltas may be negative */
typedef struct {
    uint32_t seq;
    uint64_t t_us;
    uint8_t source, flags, n_exp, n_pulses;
    int32_t exp_delta_us[GS_MAX_BURST_EXPOSURES];
    uint16_t pulse_offset_us[GS_MAX_PULSES_PER_EXPOSURE];
} gs_trigger_t;

typedef struct { uint8_t req_type; uint8_t code; uint32_t detail; } gs_result_t;
#define GS_RESULT_LEN 6u
typedef struct { uint8_t code; uint32_t value; } gs_event_t;
#define GS_EVENT_LEN 5u

typedef struct {
    gs_strobe_cfg_t cfg;
    uint32_t exposure_len_us, span_us, lockout_eff_us, duty_ppm_worst, pps_worst, latency_ns;
} gs_cfg_report_t;
#define GS_SET_CONFIG_LEN 28u
#define GS_CONFIG_REPORT_LEN 52u
#define GS_OPTIONS_LEN 12u

/* pack_* return the body length (>0) or -1; unpack_* return 0 on success, -1 on bad length. */
int gs_pack_info(const gs_info_t *m, uint8_t *b, size_t cap);
int gs_unpack_info(const uint8_t *b, size_t n, gs_info_t *m);
int gs_pack_set_config(const gs_strobe_cfg_t *m, uint8_t *b, size_t cap);
int gs_unpack_set_config(const uint8_t *b, size_t n, gs_strobe_cfg_t *m);
int gs_pack_options(const gs_opts_t *m, uint8_t *b, size_t cap);
int gs_unpack_options(const uint8_t *b, size_t n, gs_opts_t *m);
int gs_pack_cfg_report(const gs_cfg_report_t *m, uint8_t *b, size_t cap);
int gs_unpack_cfg_report(const uint8_t *b, size_t n, gs_cfg_report_t *m);
int gs_pack_status(const gs_status_t *m, uint8_t *b, size_t cap);
int gs_unpack_status(const uint8_t *b, size_t n, gs_status_t *m);
int gs_pack_trigger(const gs_trigger_t *m, uint8_t *b, size_t cap);
int gs_unpack_trigger(const uint8_t *b, size_t n, gs_trigger_t *m);
int gs_pack_result(const gs_result_t *m, uint8_t *b, size_t cap);
int gs_unpack_result(const uint8_t *b, size_t n, gs_result_t *m);
int gs_pack_event(const gs_event_t *m, uint8_t *b, size_t cap);
int gs_unpack_event(const uint8_t *b, size_t n, gs_event_t *m);

/* Name of an error code (for logs/tests). */
const char *gs_err_name(int code);

#ifdef __cplusplus
}
#endif

#endif
