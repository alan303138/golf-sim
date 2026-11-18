#include "gs_proto.h"

#include <string.h>

/* ---- little-endian writer / reader with bounds tracking ----------------- */
typedef struct { uint8_t *p; size_t cap, n; int err; } wr_t;
typedef struct { const uint8_t *p; size_t n, i; int err; } rd_t;

static void w8(wr_t *w, uint32_t v) { if (w->n + 1 > w->cap) { w->err = 1; return; } w->p[w->n++] = (uint8_t)v; }
static void w16(wr_t *w, uint32_t v) { w8(w, v); w8(w, v >> 8); }
static void w32(wr_t *w, uint32_t v) { w16(w, v); w16(w, v >> 16); }
static void w64(wr_t *w, uint64_t v) { w32(w, (uint32_t)v); w32(w, (uint32_t)(v >> 32)); }
static uint32_t r8(rd_t *r) { if (r->i + 1 > r->n) { r->err = 1; return 0; } return r->p[r->i++]; }
static uint32_t r16(rd_t *r) { uint32_t a = r8(r); return a | (r8(r) << 8); }
static uint32_t r32(rd_t *r) { uint32_t a = r16(r); return a | (r16(r) << 16); }
static uint64_t r64(rd_t *r) { uint64_t a = r32(r); return a | ((uint64_t)r32(r) << 32); }
static int wdone(const wr_t *w) { return w->err ? -1 : (int)w->n; }
static int rdone(const rd_t *r, int exact) { return (r->err || (exact && r->i != r->n)) ? -1 : 0; }

/* ---- framing -------------------------------------------------------------- */
size_t gs_frame_encode(uint8_t type, uint8_t tag, const uint8_t *body, size_t body_len,
                       uint8_t *out, size_t out_cap)
{
    if (body_len > GS_PROTO_MAX_BODY || out_cap < GS_PROTO_MAX_FRAME) return 0;
    uint8_t raw[GS_PROTO_MAX_RAW];
    raw[0] = type;
    raw[1] = tag;
    if (body_len) memcpy(&raw[2], body, body_len);
    uint16_t crc = gs_crc16(raw, 2 + body_len);
    raw[2 + body_len] = (uint8_t)crc;
    raw[3 + body_len] = (uint8_t)(crc >> 8);
    size_t e = gs_cobs_encode(raw, 4 + body_len, out, out_cap - 1);
    if (e == 0) return 0;
    out[e] = 0;
    return e + 1;
}

void gs_deframer_init(gs_deframer_t *d)
{
    memset(d, 0, sizeof(*d));
}

int gs_deframer_push(gs_deframer_t *d, uint8_t byte, gs_frame_t *out)
{
    if (byte != 0) {
        if (d->len < sizeof(d->buf)) d->buf[d->len++] = byte;
        else d->overflow = 1;
        return 0;
    }
    /* delimiter */
    size_t len = d->len;
    int ovf = d->overflow;
    d->len = 0;
    d->overflow = 0;
    if (len == 0 && !ovf) return 0;   /* empty frame: idle delimiter, ignore */
    if (ovf) { d->bad_frames++; return 0; }
    uint8_t raw[GS_PROTO_MAX_RAW];
    size_t n = gs_cobs_decode(d->buf, len, raw, sizeof(raw));
    if (n == GS_COBS_ERROR || n < 4) { d->bad_frames++; return 0; }
    uint16_t crc = (uint16_t)(raw[n - 2] | (raw[n - 1] << 8));
    if (gs_crc16(raw, n - 2) != crc) { d->bad_frames++; return 0; }
    out->type = raw[0];
    out->tag = raw[1];
    out->body_len = (uint16_t)(n - 4);
    if (out->body_len) memcpy(out->body, &raw[2], out->body_len);
    return 1;
}

/* ---- bodies ---------------------------------------------------------------- */
int gs_pack_info(const gs_info_t *m, uint8_t *b, size_t cap)
{
    wr_t w = { b, cap, 0, 0 };
    w8(&w, m->proto_major); w8(&w, m->proto_minor);
    w8(&w, m->fw_major); w8(&w, m->fw_minor); w8(&w, m->fw_patch); w8(&w, m->hw_revision);
    for (int i = 0; i < 8; i++) w8(&w, m->uid[i]);
    w32(&w, m->uptime_ms); w32(&w, m->clk_hz);
    w32(&w, m->max_width_us); w32(&w, m->min_width_us); w32(&w, m->max_duty_ppm);
    w32(&w, m->duty_window_us); w32(&w, m->max_pps);
    w8(&w, m->max_pulses_per_exposure); w8(&w, m->max_burst_exposures);
    w32(&w, m->max_exposure_us);
    return wdone(&w);
}

int gs_unpack_info(const uint8_t *b, size_t n, gs_info_t *m)
{
    rd_t r = { b, n, 0, 0 };
    m->proto_major = (uint8_t)r8(&r); m->proto_minor = (uint8_t)r8(&r);
    m->fw_major = (uint8_t)r8(&r); m->fw_minor = (uint8_t)r8(&r);
    m->fw_patch = (uint8_t)r8(&r); m->hw_revision = (uint8_t)r8(&r);
    for (int i = 0; i < 8; i++) m->uid[i] = (uint8_t)r8(&r);
    m->uptime_ms = r32(&r); m->clk_hz = r32(&r);
    m->max_width_us = r32(&r); m->min_width_us = r32(&r); m->max_duty_ppm = r32(&r);
    m->duty_window_us = r32(&r); m->max_pps = r32(&r);
    m->max_pulses_per_exposure = (uint8_t)r8(&r); m->max_burst_exposures = (uint8_t)r8(&r);
    m->max_exposure_us = r32(&r);
    return rdone(&r, 1);
}

static void put_cfg(wr_t *w, const gs_strobe_cfg_t *m)
{
    w8(w, m->mode); w8(w, m->pulses_per_exposure); w8(w, m->burst_exposures); w8(w, 0);
    w32(w, m->strobe_width_us); w32(w, m->strobe_interval_us); w32(w, m->first_pulse_delay_us);
    w32(w, m->exposure_tail_us); w32(w, m->frame_period_us); w32(w, m->trigger_delay_us);
}

static void get_cfg(rd_t *r, gs_strobe_cfg_t *m)
{
    m->mode = (uint8_t)r8(r); m->pulses_per_exposure = (uint8_t)r8(r);
    m->burst_exposures = (uint8_t)r8(r); (void)r8(r);
    m->strobe_width_us = r32(r); m->strobe_interval_us = r32(r); m->first_pulse_delay_us = r32(r);
    m->exposure_tail_us = r32(r); m->frame_period_us = r32(r); m->trigger_delay_us = r32(r);
}

int gs_pack_set_config(const gs_strobe_cfg_t *m, uint8_t *b, size_t cap)
{
    wr_t w = { b, cap, 0, 0 };
    put_cfg(&w, m);
    return wdone(&w);
}

int gs_unpack_set_config(const uint8_t *b, size_t n, gs_strobe_cfg_t *m)
{
    rd_t r = { b, n, 0, 0 };
    get_cfg(&r, m);
    return rdone(&r, 1);
}

int gs_pack_options(const gs_opts_t *m, uint8_t *b, size_t cap)
{
    wr_t w = { b, cap, 0, 0 };
    w8(&w, m->cam_active_high); w8(&w, m->trig_rising); w16(&w, m->status_period_ms);
    w32(&w, m->lockout_us); w32(&w, m->vdiv_x1000);
    return wdone(&w);
}

int gs_unpack_options(const uint8_t *b, size_t n, gs_opts_t *m)
{
    rd_t r = { b, n, 0, 0 };
    m->cam_active_high = (uint8_t)r8(&r); m->trig_rising = (uint8_t)r8(&r);
    m->status_period_ms = (uint16_t)r16(&r);
    m->lockout_us = r32(&r); m->vdiv_x1000 = r32(&r);
    return rdone(&r, 1);
}

int gs_pack_cfg_report(const gs_cfg_report_t *m, uint8_t *b, size_t cap)
{
    wr_t w = { b, cap, 0, 0 };
    put_cfg(&w, &m->cfg);
    w32(&w, m->exposure_len_us); w32(&w, m->span_us); w32(&w, m->lockout_eff_us);
    w32(&w, m->duty_ppm_worst); w32(&w, m->pps_worst); w32(&w, m->latency_ns);
    return wdone(&w);
}

int gs_unpack_cfg_report(const uint8_t *b, size_t n, gs_cfg_report_t *m)
{
    rd_t r = { b, n, 0, 0 };
    get_cfg(&r, &m->cfg);
    m->exposure_len_us = r32(&r); m->span_us = r32(&r); m->lockout_eff_us = r32(&r);
    m->duty_ppm_worst = r32(&r); m->pps_worst = r32(&r); m->latency_ns = r32(&r);
    return rdone(&r, 1);
}

int gs_pack_status(const gs_status_t *m, uint8_t *b, size_t cap)
{
    wr_t w = { b, cap, 0, 0 };
    w8(&w, m->armed); w8(&w, m->mode); w8(&w, m->interlock_ok); w8(&w, m->flags);
    w32(&w, m->faults); w32(&w, m->uptime_ms); w32(&w, m->rail_mv);
    w32(&w, m->trig_hw); w32(&w, m->trig_sw); w32(&w, m->trig_inhibited);
    w32(&w, m->bursts_done); w32(&w, m->pulses_total); w32(&w, m->rx_bad_frames);
    w32(&w, m->duty_ppm_now); w32(&w, m->max_pulse_us_seen);
    return wdone(&w);
}

int gs_unpack_status(const uint8_t *b, size_t n, gs_status_t *m)
{
    rd_t r = { b, n, 0, 0 };
    m->armed = (uint8_t)r8(&r); m->mode = (uint8_t)r8(&r);
    m->interlock_ok = (uint8_t)r8(&r); m->flags = (uint8_t)r8(&r);
    m->faults = r32(&r); m->uptime_ms = r32(&r); m->rail_mv = r32(&r);
    m->trig_hw = r32(&r); m->trig_sw = r32(&r); m->trig_inhibited = r32(&r);
    m->bursts_done = r32(&r); m->pulses_total = r32(&r); m->rx_bad_frames = r32(&r);
    m->duty_ppm_now = r32(&r); m->max_pulse_us_seen = r32(&r);
    return rdone(&r, 1);
}

int gs_pack_trigger(const gs_trigger_t *m, uint8_t *b, size_t cap)
{
    if (m->n_exp > GS_MAX_BURST_EXPOSURES || m->n_pulses > GS_MAX_PULSES_PER_EXPOSURE) return -1;
    wr_t w = { b, cap, 0, 0 };
    w32(&w, m->seq); w64(&w, m->t_us);
    w8(&w, m->source); w8(&w, m->flags); w8(&w, m->n_exp); w8(&w, m->n_pulses);
    for (unsigned i = 0; i < m->n_exp; i++) w32(&w, (uint32_t)m->exp_delta_us[i]);
    for (unsigned i = 0; i < m->n_pulses; i++) w16(&w, m->pulse_offset_us[i]);
    return wdone(&w);
}

int gs_unpack_trigger(const uint8_t *b, size_t n, gs_trigger_t *m)
{
    rd_t r = { b, n, 0, 0 };
    memset(m, 0, sizeof(*m));
    m->seq = r32(&r); m->t_us = r64(&r);
    m->source = (uint8_t)r8(&r); m->flags = (uint8_t)r8(&r);
    m->n_exp = (uint8_t)r8(&r); m->n_pulses = (uint8_t)r8(&r);
    if (m->n_exp > GS_MAX_BURST_EXPOSURES || m->n_pulses > GS_MAX_PULSES_PER_EXPOSURE) return -1;
    for (unsigned i = 0; i < m->n_exp; i++) m->exp_delta_us[i] = (int32_t)r32(&r);
    for (unsigned i = 0; i < m->n_pulses; i++) m->pulse_offset_us[i] = (uint16_t)r16(&r);
    return rdone(&r, 1);
}

int gs_pack_result(const gs_result_t *m, uint8_t *b, size_t cap)
{
    wr_t w = { b, cap, 0, 0 };
    w8(&w, m->req_type); w8(&w, m->code); w32(&w, m->detail);
    return wdone(&w);
}

int gs_unpack_result(const uint8_t *b, size_t n, gs_result_t *m)
{
    rd_t r = { b, n, 0, 0 };
    m->req_type = (uint8_t)r8(&r); m->code = (uint8_t)r8(&r); m->detail = r32(&r);
    return rdone(&r, 1);
}

int gs_pack_event(const gs_event_t *m, uint8_t *b, size_t cap)
{
    wr_t w = { b, cap, 0, 0 };
    w8(&w, m->code); w32(&w, m->value);
    return wdone(&w);
}

int gs_unpack_event(const uint8_t *b, size_t n, gs_event_t *m)
{
    rd_t r = { b, n, 0, 0 };
    m->code = (uint8_t)r8(&r); m->value = r32(&r);
    return rdone(&r, 1);
}

const char *gs_err_name(int c)
{
    switch (c) {
    case GS_OK: return "OK";
    case GS_ERR_UNKNOWN_CMD: return "UNKNOWN_CMD";
    case GS_ERR_BAD_LENGTH: return "BAD_LENGTH";
    case GS_ERR_BAD_VALUE: return "BAD_VALUE";
    case GS_ERR_LIMIT_WIDTH: return "LIMIT_WIDTH";
    case GS_ERR_LIMIT_DUTY: return "LIMIT_DUTY";
    case GS_ERR_LIMIT_PPS: return "LIMIT_PPS";
    case GS_ERR_LIMIT_TIMING: return "LIMIT_TIMING";
    case GS_ERR_BUSY_ARMED: return "BUSY_ARMED";
    case GS_ERR_NOT_ARMED: return "NOT_ARMED";
    case GS_ERR_FAULT_LATCHED: return "FAULT_LATCHED";
    case GS_ERR_LOCKOUT: return "LOCKOUT";
    case GS_ERR_BAD_MAGIC: return "BAD_MAGIC";
    case GS_ERR_HW: return "HW";
    default: return "UNKNOWN";
    }
}
