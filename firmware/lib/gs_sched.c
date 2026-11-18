#include "gs_sched.h"

#include <string.h>

void gs_cfg_default(gs_strobe_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->mode = GS_MODE_BURST;
    c->strobe_width_us = 10;
    c->strobe_interval_us = 700;
    c->pulses_per_exposure = 5;
    c->first_pulse_delay_us = 50;
    c->exposure_tail_us = 50;
    c->burst_exposures = 2;
    c->frame_period_us = 16700;
    c->trigger_delay_us = 0;
}

void gs_opts_default(gs_opts_t *o)
{
    memset(o, 0, sizeof(*o));
    o->cam_active_high = GS_DEFAULT_CAM_ACTIVE_HIGH;
    o->trig_rising = GS_DEFAULT_TRIG_RISING;
    o->status_period_ms = GS_DEFAULT_STATUS_PERIOD_MS;
    o->lockout_us = GS_DEFAULT_LOCKOUT_US;
    o->vdiv_x1000 = GS_DEFAULT_VDIV_X1000;
}

int gs_opts_validate(const gs_opts_t *o, uint32_t *detail)
{
    uint32_t d = 0;
    int e = GS_OK;
    if (o->cam_active_high > 1 || o->trig_rising > 1) {
        e = GS_ERR_BAD_VALUE; d = GS_FIELD_POLARITY;
    } else if (o->lockout_us > GS_MAX_LOCKOUT_US) {
        e = GS_ERR_BAD_VALUE; d = GS_FIELD_LOCKOUT;
    } else if (o->vdiv_x1000 == 0 || o->vdiv_x1000 > 1000000u) {
        e = GS_ERR_BAD_VALUE; d = GS_FIELD_VDIV;
    } else if (o->status_period_ms != 0 && o->status_period_ms < 10) {
        e = GS_ERR_BAD_VALUE; d = GS_FIELD_STATUS_PERIOD;
    }
    if (detail) *detail = d;
    return e;
}

uint32_t gs_wave_word(bool cam, bool strobe, bool last, uint32_t cycles)
{
    uint32_t x = (cycles >= GS_SEG_OVERHEAD_CYCLES) ? cycles - GS_SEG_OVERHEAD_CYCLES : 0;
    return (cam ? 1u : 0u) | (strobe ? 2u : 0u) | (last ? 4u : 0u) | (x << 3);
}

uint32_t gs_wave_word_cycles(uint32_t w)
{
    return (w >> 3) + GS_SEG_OVERHEAD_CYCLES;
}

uint32_t gs_max_pulses_in_window(const uint32_t *st, uint32_t n, uint32_t period, uint32_t window)
{
    if (n == 0 || window == 0 || period == 0) return 0;
    uint64_t best = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint64_t cnt = 0;
        for (uint32_t j = 0; j < n; j++) {
            if (st[j] >= st[i]) {
                uint64_t diff = st[j] - st[i];
                if (diff < window) cnt += (window - 1 - diff) / period + 1;
            } else {
                uint64_t d2 = st[i] - st[j];
                cnt += (window - 1 + d2) / period;
            }
        }
        if (cnt > best) best = cnt;
    }
    return best > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)best;
}

uint32_t gs_duty_ppm(uint32_t count, uint32_t width_us, uint32_t window_us)
{
    if (window_us == 0) return 0xFFFFFFFFu;
    uint64_t num = (uint64_t)count * width_us * 1000000ull;
    uint64_t r = (num + window_us - 1) / window_us;
    return r > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)r;
}

uint32_t gs_rail_mv(uint32_t raw, uint32_t vref_mv, uint32_t vdiv_x1000)
{
    uint64_t v = (uint64_t)raw * vref_mv * vdiv_x1000;
    return (uint32_t)((v + (4096ull * 1000ull) / 2) / (4096ull * 1000ull));
}

static int fail(gs_sched_t *o, int err, uint32_t detail)
{
    o->err_detail = detail;
    return err;
}

typedef struct {
    gs_sched_t *s;
    uint32_t n;
    int overflow;
} emitter_t;

static void emit(emitter_t *e, bool cam, bool strobe, uint64_t cycles)
{
    if (e->n >= GS_MAX_WAVE_WORDS) { e->overflow = 1; return; }
    e->s->words[e->n++] = gs_wave_word(cam, strobe, false, (uint32_t)cycles);
}

int gs_sched_build(const gs_strobe_cfg_t *c, const gs_opts_t *opts, gs_sched_t *s)
{
    uint32_t d = 0;
    int err;
    memset(s, 0, sizeof(*s));

    if (c->mode > GS_MODE_FREE_RUN) return fail(s, GS_ERR_BAD_VALUE, GS_FIELD_MODE);
    if (c->strobe_width_us < GS_MIN_STROBE_WIDTH_US || c->strobe_width_us > GS_MAX_STROBE_WIDTH_US)
        return fail(s, GS_ERR_LIMIT_WIDTH, c->strobe_width_us);
    if (c->pulses_per_exposure < 1 || c->pulses_per_exposure > GS_MAX_PULSES_PER_EXPOSURE)
        return fail(s, GS_ERR_BAD_VALUE, GS_FIELD_PULSES);
    if (c->burst_exposures < 1 || c->burst_exposures > GS_MAX_BURST_EXPOSURES)
        return fail(s, GS_ERR_BAD_VALUE, GS_FIELD_BURST_EXPOSURES);
    const uint64_t np = c->pulses_per_exposure;
    const uint64_t width = c->strobe_width_us;
    if (np > 1 && (uint64_t)c->strobe_interval_us < width + GS_MIN_STROBE_GAP_US)
        return fail(s, GS_ERR_LIMIT_TIMING, GS_FIELD_INTERVAL);
    if (c->first_pulse_delay_us < GS_MIN_SEG_US) return fail(s, GS_ERR_BAD_VALUE, GS_FIELD_FIRST_DELAY);
    if (c->exposure_tail_us < GS_MIN_SEG_US) return fail(s, GS_ERR_BAD_VALUE, GS_FIELD_TAIL);
    const uint64_t interval = np > 1 ? c->strobe_interval_us : 0;
    const uint64_t E = (uint64_t)c->first_pulse_delay_us + (np - 1) * interval + width + c->exposure_tail_us;
    if (E > GS_MAX_EXPOSURE_US) return fail(s, GS_ERR_LIMIT_TIMING, GS_FIELD_EXPOSURE);
    const bool free_run = (c->mode == GS_MODE_FREE_RUN);
    const uint32_t B = free_run ? 1u : c->burst_exposures;
    if (free_run || c->burst_exposures > 1) {
        if (c->frame_period_us > GS_MAX_FRAME_PERIOD_US ||
            (uint64_t)c->frame_period_us < E + GS_MIN_EXPOSURE_GAP_US)
            return fail(s, GS_ERR_LIMIT_TIMING, GS_FIELD_FRAME_PERIOD);
    }
    if (c->trigger_delay_us > GS_MAX_TRIGGER_DELAY_US) return fail(s, GS_ERR_BAD_VALUE, GS_FIELD_TRIGGER_DELAY);
    err = gs_opts_validate(opts, &d);
    if (err) return fail(s, err, d);

    s->exposure_len_us = (uint32_t)E;
    s->n_exposures = B;
    s->n_pulses = (uint32_t)np;
    for (uint32_t p = 0; p < np; p++)
        s->pulse_offsets_us[p] = (uint16_t)(c->first_pulse_delay_us + p * interval);
    for (uint32_t e = 0; e < B; e++)
        s->exposure_start_rel_us[e] = free_run ? 0 : c->trigger_delay_us + e * c->frame_period_us;
    s->latency_ns = GS_TRIG_LATENCY_CYCLES * 1000u / GS_CYCLES_PER_US;

    /* ---- build words (without the final idle word and padding) ---- */
    emitter_t em = { s, 0, 0 };
    const uint64_t cpu = GS_CYCLES_PER_US;
    if (!free_run && c->trigger_delay_us > 0) emit(&em, false, false, c->trigger_delay_us * cpu);
    for (uint32_t e = 0; e < B; e++) {
        emit(&em, true, false, c->first_pulse_delay_us * cpu);
        for (uint32_t p = 0; p < np; p++) {
            emit(&em, true, true, width * cpu);
            emit(&em, true, false, (p + 1 < np ? interval - width : (uint64_t)c->exposure_tail_us) * cpu);
        }
        if (e + 1 < B) emit(&em, false, false, ((uint64_t)c->frame_period_us - E) * cpu);
    }
    if (em.overflow || em.n + 1 > GS_MAX_WAVE_WORDS) return fail(s, GS_ERR_LIMIT_TIMING, GS_FIELD_EXPOSURE);

    /* sum of cycles of the real words so far */
    uint64_t cycles = 0;
    for (uint32_t i = 0; i < em.n; i++) cycles += gs_wave_word_cycles(s->words[i]);

    /* ---- final idle word + power-of-two padding (idle words, 6 cycles each) ---- */
    uint32_t n_real = em.n + 1;
    uint32_t ring = GS_MIN_WAVE_WORDS;
    while (ring < n_real) ring <<= 1;
    if (ring > GS_MAX_WAVE_WORDS) return fail(s, GS_ERR_LIMIT_TIMING, GS_FIELD_EXPOSURE);
    uint32_t n_pad = ring - n_real;
    for (uint32_t i = 0; i < n_pad; i++) emit(&em, false, false, GS_SEG_OVERHEAD_CYCLES);
    cycles += (uint64_t)n_pad * GS_SEG_OVERHEAD_CYCLES;
    if (free_run) {
        /* The frame must be exactly frame_period long: the last idle word absorbs the rest. */
        uint64_t frame_cycles = (uint64_t)c->frame_period_us * cpu;
        uint64_t last = frame_cycles - cycles;   /* frame >= E + 100 us, so this is large */
        s->words[em.n++] = gs_wave_word(false, false, false, (uint32_t)last);
        cycles += last;
    } else {
        s->words[em.n++] = gs_wave_word(false, false, true, GS_FINAL_IDLE_CYCLES);
        cycles += GS_FINAL_IDLE_CYCLES;
    }
    s->n_words = em.n;
    s->n_real = n_real;
    s->span_cycles = cycles;
    s->span_us = (uint32_t)((cycles + cpu - 1) / cpu);

    /* ---- lockout and limit analysis ---- */
    uint32_t period;
    if (free_run) {
        s->lockout_eff_us = opts->lockout_us;
        period = c->frame_period_us;
    } else {
        uint32_t min_lock = s->span_us + GS_LOCKOUT_MARGIN_US;
        s->lockout_eff_us = opts->lockout_us > min_lock ? opts->lockout_us : min_lock;
        period = s->lockout_eff_us;
    }
    s->period_us = period;

    static uint32_t starts[GS_MAX_BURST_EXPOSURES * GS_MAX_PULSES_PER_EXPOSURE];
    uint32_t n = 0;
    for (uint32_t e = 0; e < B; e++)
        for (uint32_t p = 0; p < np; p++)
            starts[n++] = s->exposure_start_rel_us[e] + s->pulse_offsets_us[p];
    uint32_t cnt_w = gs_max_pulses_in_window(starts, n, period, GS_DUTY_WINDOW_US);
    uint32_t cnt_s = gs_max_pulses_in_window(starts, n, period, GS_PPS_WINDOW_US);
    s->duty_ppm_worst = gs_duty_ppm(cnt_w, c->strobe_width_us, GS_DUTY_WINDOW_US);
    s->pps_worst = cnt_s;
    if ((uint64_t)cnt_w * c->strobe_width_us * 1000000ull > (uint64_t)GS_MAX_DUTY_PPM * GS_DUTY_WINDOW_US)
        return fail(s, GS_ERR_LIMIT_DUTY, s->duty_ppm_worst);
    if (cnt_s > GS_MAX_PULSES_PER_SEC) return fail(s, GS_ERR_LIMIT_PPS, cnt_s);
    return GS_OK;
}
