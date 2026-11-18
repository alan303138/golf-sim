#include "gs_core.h"

#include <string.h>

#include "gs_version.h"

static void send(gs_core_t *c, uint8_t type, uint8_t tag, const uint8_t *body, size_t n)
{
    uint8_t out[GS_PROTO_MAX_FRAME];
    size_t len = gs_frame_encode(type, tag, body, n, out, sizeof(out));
    if (len) c->hal.tx(c->ctx, out, len);
}

static void send_result(gs_core_t *c, uint8_t req, uint8_t tag, int code, uint32_t detail)
{
    uint8_t b[GS_RESULT_LEN];
    gs_result_t r = { req, (uint8_t)code, detail };
    int n = gs_pack_result(&r, b, sizeof(b));
    if (n > 0) send(c, GS_MSG_RESULT, tag, b, (size_t)n);
}

static void send_event(gs_core_t *c, uint8_t code, uint32_t value)
{
    uint8_t b[GS_EVENT_LEN];
    gs_event_t e = { code, value };
    int n = gs_pack_event(&e, b, sizeof(b));
    if (n > 0) send(c, GS_MSG_EVENT, 0, b, (size_t)n);
}

static void send_info(gs_core_t *c, uint8_t tag)
{
    gs_info_t m;
    memset(&m, 0, sizeof(m));
    m.proto_major = GS_PROTO_VERSION_MAJOR; m.proto_minor = GS_PROTO_VERSION_MINOR;
    m.fw_major = GS_FW_VERSION_MAJOR; m.fw_minor = GS_FW_VERSION_MINOR; m.fw_patch = GS_FW_VERSION_PATCH;
    m.hw_revision = GS_HW_REVISION;
    c->hal.hw_uid(c->ctx, m.uid);
    m.uptime_ms = (uint32_t)(c->hal.now_us(c->ctx) / 1000u);
    m.clk_hz = GS_CLK_HZ;
    m.max_width_us = GS_MAX_STROBE_WIDTH_US; m.min_width_us = GS_MIN_STROBE_WIDTH_US;
    m.max_duty_ppm = GS_MAX_DUTY_PPM; m.duty_window_us = GS_DUTY_WINDOW_US; m.max_pps = GS_MAX_PULSES_PER_SEC;
    m.max_pulses_per_exposure = GS_MAX_PULSES_PER_EXPOSURE; m.max_burst_exposures = GS_MAX_BURST_EXPOSURES;
    m.max_exposure_us = GS_MAX_EXPOSURE_US;
    uint8_t b[GS_INFO_LEN];
    int n = gs_pack_info(&m, b, sizeof(b));
    if (n > 0) send(c, GS_MSG_INFO, tag, b, (size_t)n);
}

static void send_config(gs_core_t *c, uint8_t tag)
{
    gs_cfg_report_t m;
    m.cfg = c->cfg;
    m.exposure_len_us = c->sched.exposure_len_us;
    m.span_us = c->sched.span_us;
    m.lockout_eff_us = c->sched.lockout_eff_us;
    m.duty_ppm_worst = c->sched.duty_ppm_worst;
    m.pps_worst = c->sched.pps_worst;
    m.latency_ns = c->sched.latency_ns;
    uint8_t b[GS_CONFIG_REPORT_LEN];
    int n = gs_pack_cfg_report(&m, b, sizeof(b));
    if (n > 0) send(c, GS_MSG_CONFIG, tag, b, (size_t)n);
}

static void send_options(gs_core_t *c, uint8_t tag)
{
    uint8_t b[GS_OPTIONS_LEN];
    int n = gs_pack_options(&c->opts, b, sizeof(b));
    if (n > 0) send(c, GS_MSG_OPTIONS, tag, b, (size_t)n);
}

static void send_status(gs_core_t *c, uint8_t tag)
{
    gs_status_t s;
    memset(&s, 0, sizeof(s));
    s.armed = c->armed;
    s.mode = c->cfg.mode;
    s.interlock_ok = c->interlock_ok;
    s.flags = c->interlock_ok ? (uint8_t)0 : (uint8_t)GS_STATUS_FLAG_STROBE_FORCED_LOW;
    s.faults = c->faults;
    uint64_t now = c->hal.now_us(c->ctx);
    s.uptime_ms = (uint32_t)(now / 1000u);
    s.rail_mv = gs_rail_mv(c->hal.hw_rail_raw(c->ctx), GS_ADC_VREF_MV, c->opts.vdiv_x1000);
    s.trig_hw = c->trig_hw; s.trig_sw = c->trig_sw; s.trig_inhibited = c->trig_inhibited;
    s.bursts_done = c->bursts_done; s.rx_bad_frames = c->rx_bad;
    c->hal.hw_stats(c->ctx, &s.pulses_total, &s.max_pulse_us_seen, &s.duty_ppm_now);
    uint8_t b[GS_STATUS_LEN];
    int n = gs_pack_status(&s, b, sizeof(b));
    if (n > 0) send(c, GS_MSG_STATUS, tag, b, (size_t)n);
    c->last_status_us = now;
}

void gs_core_init(gs_core_t *c, const gs_hal_t *hal, void *ctx, int wdt_reset)
{
    memset(c, 0, sizeof(*c));
    c->hal = *hal;
    c->ctx = ctx;
    gs_cfg_default(&c->cfg);
    gs_opts_default(&c->opts);
    int e = gs_sched_build(&c->cfg, &c->opts, &c->sched);
    (void)e; /* defaults are validated by the unit tests */
    c->interlock_ok = (uint8_t)(c->hal.hw_interlock_ok(ctx) != 0);
    c->hal.hw_apply_options(ctx, &c->opts);
    c->last_status_us = c->hal.now_us(ctx);
    if (wdt_reset) c->faults |= GS_FAULT_WDT_RESET;
    send_event(c, GS_EVT_BOOT, wdt_reset ? 1u : 0u);
}

void gs_core_set_rx_bad(gs_core_t *c, uint32_t n) { c->rx_bad = n; }

void gs_core_fault(gs_core_t *c, uint32_t bits)
{
    uint32_t fresh = bits & ~c->faults;
    c->faults |= bits;
    if (bits & GS_FAULT_STOPPING) {
        c->hal.hw_stop(c->ctx);
        if (c->armed) {
            c->armed = 0;
            send_event(c, GS_EVT_DISARMED, bits);
        }
    }
    if (fresh) send_event(c, GS_EVT_FAULT, fresh);
}

void gs_core_interlock(gs_core_t *c, int ok)
{
    ok = ok != 0;
    if (ok == c->interlock_ok) return;
    c->interlock_ok = (uint8_t)ok;
    if (!ok) c->faults |= GS_FAULT_INTERLOCK_TRIPPED;
    send_event(c, GS_EVT_INTERLOCK, (uint32_t)ok);
}

void gs_core_burst_done(gs_core_t *c) { c->bursts_done++; }

void gs_core_trigger(gs_core_t *c, uint64_t t_us, int software, int inhibited)
{
    if (!c->armed) return;
    gs_trigger_t m;
    memset(&m, 0, sizeof(m));
    c->seq++;
    m.seq = c->seq;
    m.t_us = t_us;
    m.source = software ? GS_TRIG_SRC_SOFTWARE : GS_TRIG_SRC_HARDWARE;
    if (software) c->trig_sw++; else c->trig_hw++;
    c->last_trig_us = t_us;
    c->have_trig = 1;
    if (inhibited) { m.flags |= GS_TRIG_FLAG_INHIBITED; c->trig_inhibited++; }
    if (c->cfg.mode == GS_MODE_BURST) {
        m.n_exp = (uint8_t)c->sched.n_exposures;
        for (unsigned i = 0; i < m.n_exp; i++) m.exp_delta_us[i] = (int32_t)c->sched.exposure_start_rel_us[i];
    } else {
        /* free_run: report the last exposure that started at or before the event and the next one */
        const uint64_t P = c->cfg.frame_period_us;
        uint64_t k = t_us >= c->epoch_us ? (t_us - c->epoch_us) / P : 0;
        int64_t d0 = (int64_t)(c->epoch_us + k * P) - (int64_t)t_us;
        m.flags |= GS_TRIG_FLAG_FREE_RUN;
        m.n_exp = 2;
        m.exp_delta_us[0] = (int32_t)d0;
        m.exp_delta_us[1] = (int32_t)(d0 + (int64_t)P);
    }
    if (!inhibited) {
        m.n_pulses = (uint8_t)c->sched.n_pulses;
        for (unsigned i = 0; i < m.n_pulses; i++) m.pulse_offset_us[i] = c->sched.pulse_offsets_us[i];
    }
    uint8_t b[GS_PROTO_MAX_BODY];
    int n = gs_pack_trigger(&m, b, sizeof(b));
    if (n > 0) send(c, GS_MSG_TRIGGER, 0, b, (size_t)n);
}

void gs_core_tick(gs_core_t *c)
{
    if (c->opts.status_period_ms == 0) return;
    uint64_t now = c->hal.now_us(c->ctx);
    if (now - c->last_status_us >= (uint64_t)c->opts.status_period_ms * 1000u) send_status(c, 0);
}

static void handle_set_config(gs_core_t *c, const gs_frame_t *f)
{
    gs_strobe_cfg_t cfg;
    if (gs_unpack_set_config(f->body, f->body_len, &cfg) != 0) {
        send_result(c, f->type, f->tag, GS_ERR_BAD_LENGTH, f->body_len);
        return;
    }
    if (c->armed) { send_result(c, f->type, f->tag, GS_ERR_BUSY_ARMED, 0); return; }
    int e = gs_sched_build(&cfg, &c->opts, &c->scratch);
    if (e) { send_result(c, f->type, f->tag, e, c->scratch.err_detail); return; }
    c->cfg = cfg;
    memcpy(&c->sched, &c->scratch, sizeof(c->sched));
    send_config(c, f->tag);
}

static void handle_set_options(gs_core_t *c, const gs_frame_t *f)
{
    gs_opts_t o;
    if (gs_unpack_options(f->body, f->body_len, &o) != 0) {
        send_result(c, f->type, f->tag, GS_ERR_BAD_LENGTH, f->body_len);
        return;
    }
    if (c->armed) { send_result(c, f->type, f->tag, GS_ERR_BUSY_ARMED, 0); return; }
    int e = gs_sched_build(&c->cfg, &o, &c->scratch);
    if (e) { send_result(c, f->type, f->tag, e, c->scratch.err_detail); return; }
    c->opts = o;
    memcpy(&c->sched, &c->scratch, sizeof(c->sched));
    c->hal.hw_apply_options(c->ctx, &c->opts);
    send_result(c, f->type, f->tag, GS_OK, 0);
}

static void handle_arm(gs_core_t *c, const gs_frame_t *f)
{
    if (f->body_len != 1 || f->body[0] > 1) { send_result(c, f->type, f->tag, GS_ERR_BAD_LENGTH, f->body_len); return; }
    if (f->body[0] == 0) {
        c->hal.hw_stop(c->ctx);
        c->armed = 0;
        send_result(c, f->type, f->tag, GS_OK, 0);
        return;
    }
    if (c->armed) { send_result(c, f->type, f->tag, GS_OK, c->interlock_ok ? 0u : 1u); return; }
    if (c->faults & GS_FAULT_STOPPING) { send_result(c, f->type, f->tag, GS_ERR_FAULT_LATCHED, c->faults); return; }
    if (c->hal.hw_prepare(c->ctx, &c->sched, &c->opts, c->cfg.mode) != 0) {
        send_result(c, f->type, f->tag, GS_ERR_HW, 0);
        return;
    }
    if (c->hal.hw_start(c->ctx, &c->epoch_us) != 0) {
        send_result(c, f->type, f->tag, GS_ERR_HW, 1);
        return;
    }
    c->armed = 1;
    c->have_trig = 0;
    /* detail 1 = armed, but the strobe is inhibited because STROBE_EN is low */
    send_result(c, f->type, f->tag, GS_OK, c->interlock_ok ? 0u : 1u);
}

static void handle_fire(gs_core_t *c, const gs_frame_t *f)
{
    if (!c->armed) { send_result(c, f->type, f->tag, GS_ERR_NOT_ARMED, 0); return; }
    if (c->faults & GS_FAULT_STOPPING) { send_result(c, f->type, f->tag, GS_ERR_FAULT_LATCHED, c->faults); return; }
    uint64_t now = c->hal.now_us(c->ctx);
    if (c->have_trig) {
        uint64_t until = c->last_trig_us + c->sched.lockout_eff_us;
        if (now < until) { send_result(c, f->type, f->tag, GS_ERR_LOCKOUT, (uint32_t)(until - now)); return; }
    }
    c->hal.hw_fire(c->ctx);
    send_result(c, f->type, f->tag, GS_OK, 0);
}

void gs_core_rx_frame(gs_core_t *c, const gs_frame_t *f)
{
    switch (f->type) {
    case GS_MSG_GET_INFO: send_info(c, f->tag); break;
    case GS_MSG_SET_CONFIG: handle_set_config(c, f); break;
    case GS_MSG_SET_OPTIONS: handle_set_options(c, f); break;
    case GS_MSG_ARM: handle_arm(c, f); break;
    case GS_MSG_FIRE: handle_fire(c, f); break;
    case GS_MSG_GET_STATUS: send_status(c, f->tag); break;
    case GS_MSG_GET_CONFIG: send_config(c, f->tag); break;
    case GS_MSG_GET_OPTIONS: send_options(c, f->tag); break;
    case GS_MSG_CLEAR_FAULTS:
        c->faults = 0;
        send_result(c, f->type, f->tag, GS_OK, 0);
        break;
    case GS_MSG_BOOTLOADER: {
        uint32_t magic = 0;
        if (f->body_len != 4) { send_result(c, f->type, f->tag, GS_ERR_BAD_LENGTH, f->body_len); break; }
        for (int i = 3; i >= 0; i--) magic = (magic << 8) | f->body[i];
        if (magic != GS_BOOTLOADER_MAGIC) { send_result(c, f->type, f->tag, GS_ERR_BAD_MAGIC, 0); break; }
        c->hal.hw_stop(c->ctx);
        c->armed = 0;
        send_result(c, f->type, f->tag, GS_OK, 0);
        c->hal.hw_bootloader(c->ctx);   /* does not return on hardware */
        break;
    }
    default:
        send_result(c, f->type, f->tag, GS_ERR_UNKNOWN_CMD, f->type);
        break;
    }
}
