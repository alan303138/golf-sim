#include <string.h>

#include "gs_sched.h"
#include "gs_test.h"

static gs_sched_t S;

typedef struct {
    uint64_t cam_on[GS_MAX_BURST_EXPOSURES], cam_off[GS_MAX_BURST_EXPOSURES];
    uint64_t st_on[300], st_off[300];
    unsigned n_cam, n_st;
    uint64_t total;
    unsigned last_words;
} timeline_t;

/* Replay the words like the PIO does: pins change at the start of each word. */
static void replay(const gs_sched_t *s, timeline_t *t)
{
    memset(t, 0, sizeof(*t));
    uint64_t now = 0;
    int cam = 0, st = 0;
    for (uint32_t i = 0; i < s->n_words; i++) {
        uint32_t w = s->words[i];
        int ncam = w & 1, nst = (w >> 1) & 1;
        if (ncam && !cam) t->cam_on[t->n_cam] = now;
        if (!ncam && cam) t->cam_off[t->n_cam++] = now;
        if (nst && !st) t->st_on[t->n_st] = now;
        if (!nst && st) t->st_off[t->n_st++] = now;
        cam = ncam; st = nst;
        if (w & 4) t->last_words++;
        now += gs_wave_word_cycles(w);
    }
    t->total = now;
}

static void test_defaults(void)
{
    gs_strobe_cfg_t c; gs_opts_t o;
    gs_cfg_default(&c); gs_opts_default(&o);
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    CHECK_EQ(S.exposure_len_us, 2910);
    CHECK_EQ(S.n_exposures, 2);
    CHECK_EQ(S.n_pulses, 5);
    const uint16_t offs[5] = { 50, 750, 1450, 2150, 2850 };
    for (int i = 0; i < 5; i++) CHECK_EQ(S.pulse_offsets_us[i], offs[i]);
    CHECK_EQ(S.exposure_start_rel_us[0], 0);
    CHECK_EQ(S.exposure_start_rel_us[1], 16700);
    CHECK(S.n_words >= GS_MIN_WAVE_WORDS && (S.n_words & (S.n_words - 1)) == 0);
    CHECK_EQ(S.lockout_eff_us, 500000);
    CHECK(S.duty_ppm_worst < GS_MAX_DUTY_PPM);
    CHECK_EQ(S.latency_ns, 64);

    timeline_t t;
    replay(&S, &t);
    CHECK_EQ(t.n_cam, 2);
    CHECK_EQ(t.n_st, 10);
    CHECK_EQ(t.last_words, 1);
    CHECK_EQ(t.total, S.span_cycles);
    CHECK_EQ(S.words[S.n_words - 1] & 4, 4);
    for (int e = 0; e < 2; e++) {
        CHECK_EQ(t.cam_on[e], (uint64_t)S.exposure_start_rel_us[e] * 125);
        CHECK_EQ(t.cam_off[e] - t.cam_on[e], 2910ull * 125);
        for (int p = 0; p < 5; p++) {
            uint64_t on = t.st_on[e * 5 + p], off = t.st_off[e * 5 + p];
            CHECK_EQ(on, t.cam_on[e] + (uint64_t)offs[p] * 125);
            CHECK_EQ(off - on, 10ull * 125); /* pulse width exact to the cycle */
            if (p) CHECK_EQ(on - t.st_on[e * 5 + p - 1], 700ull * 125); /* spacing exact */
        }
    }
    CHECK(S.span_us >= 16700 + 2910);
    CHECK(S.lockout_eff_us >= S.span_us + GS_LOCKOUT_MARGIN_US);
}

static void test_trigger_delay_and_polarity_independent(void)
{
    gs_strobe_cfg_t c; gs_opts_t o;
    gs_cfg_default(&c); gs_opts_default(&o);
    c.trigger_delay_us = 1234;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    timeline_t t;
    replay(&S, &t);
    CHECK_EQ(t.cam_on[0], 1234ull * 125);
    CHECK_EQ(S.exposure_start_rel_us[1], 1234 + 16700);
    CHECK_EQ(S.words[0] & 3, 0);
    uint32_t w1[GS_MAX_WAVE_WORDS];
    memcpy(w1, S.words, sizeof(w1));
    o.cam_active_high = 1; o.trig_rising = 0;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    CHECK(memcmp(w1, S.words, sizeof(w1)) == 0); /* polarity is applied at the pins, not in the words */
    c.trigger_delay_us = GS_MAX_TRIGGER_DELAY_US;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    c.trigger_delay_us = GS_MAX_TRIGGER_DELAY_US + 1;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_BAD_VALUE);
    CHECK_EQ(S.err_detail, GS_FIELD_TRIGGER_DELAY);
}

static void test_free_run(void)
{
    gs_strobe_cfg_t c; gs_opts_t o;
    gs_cfg_default(&c); gs_opts_default(&o);
    c.mode = GS_MODE_FREE_RUN;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    timeline_t t;
    replay(&S, &t);
    CHECK_EQ(S.n_exposures, 1);
    CHECK_EQ(t.total, 16700ull * 125);        /* frame exactly frame_period */
    CHECK_EQ(t.last_words, 0);                /* never raises the end-of-burst flag */
    CHECK_EQ(t.cam_on[0], 0);
    CHECK_EQ(t.n_st, 5);
    CHECK_EQ(S.period_us, 16700);
    CHECK_EQ(S.lockout_eff_us, 500000);       /* lockout is just a debounce here */
    /* frame period boundary: E + 100 ok, E + 99 rejected */
    c.frame_period_us = 2910 + GS_MIN_EXPOSURE_GAP_US;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    replay(&S, &t);
    CHECK_EQ(t.total, (uint64_t)c.frame_period_us * 125);
    c.frame_period_us -= 1;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_LIMIT_TIMING);
    CHECK_EQ(S.err_detail, GS_FIELD_FRAME_PERIOD);
    /* 60 fps with many pulses: total is exact for several shapes */
    for (unsigned n = 1; n <= 32; n += 7) {
        gs_cfg_default(&c);
        c.mode = GS_MODE_FREE_RUN; c.pulses_per_exposure = (uint8_t)n; c.strobe_interval_us = 300;
        c.frame_period_us = 16667;
        int e = gs_sched_build(&c, &o, &S);
        CHECK(e == GS_OK || e == GS_ERR_LIMIT_DUTY);
        if (e == GS_OK) { replay(&S, &t); CHECK_EQ(t.total, 16667ull * 125); }
    }
}

static void test_limits(void)
{
    gs_strobe_cfg_t c; gs_opts_t o;
    gs_opts_default(&o);
    gs_cfg_default(&c);
    c.strobe_width_us = GS_MAX_STROBE_WIDTH_US;      /* 20: accepted */
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    c.strobe_width_us = GS_MAX_STROBE_WIDTH_US + 1;  /* 21: rejected, never clipped */
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_LIMIT_WIDTH);
    CHECK_EQ(S.err_detail, 21);
    c.strobe_width_us = 100;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_LIMIT_WIDTH);
    c.strobe_width_us = GS_MIN_STROBE_WIDTH_US;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    c.strobe_width_us = GS_MIN_STROBE_WIDTH_US - 1;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_LIMIT_WIDTH);
    CHECK_EQ(GS_MAX_STROBE_WIDTH_US, 20);

    gs_cfg_default(&c);
    c.mode = 2; CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_BAD_VALUE);
    gs_cfg_default(&c); c.pulses_per_exposure = 0; CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_BAD_VALUE);
    c.pulses_per_exposure = 33; CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_BAD_VALUE);
    gs_cfg_default(&c); c.burst_exposures = 0; CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_BAD_VALUE);
    c.burst_exposures = 9; CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_BAD_VALUE);
    gs_cfg_default(&c); c.first_pulse_delay_us = 0; CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_BAD_VALUE);
    gs_cfg_default(&c); c.exposure_tail_us = 0; CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_BAD_VALUE);

    /* interval boundary: width + min gap accepted, one less rejected */
    gs_cfg_default(&c);
    c.strobe_interval_us = c.strobe_width_us + GS_MIN_STROBE_GAP_US;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    c.strobe_interval_us -= 1;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_LIMIT_TIMING);
    CHECK_EQ(S.err_detail, GS_FIELD_INTERVAL);
    /* single pulse: interval is irrelevant */
    c.pulses_per_exposure = 1; c.strobe_interval_us = 0;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);

    /* exposure length boundary */
    gs_cfg_default(&c);
    c.burst_exposures = 1; c.pulses_per_exposure = 1;
    c.first_pulse_delay_us = 1; c.exposure_tail_us = GS_MAX_EXPOSURE_US - 1 - c.strobe_width_us;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    CHECK_EQ(S.exposure_len_us, GS_MAX_EXPOSURE_US);
    c.exposure_tail_us++;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_LIMIT_TIMING);
    CHECK_EQ(S.err_detail, GS_FIELD_EXPOSURE);
    /* huge values must not overflow into acceptance */
    gs_cfg_default(&c); c.strobe_interval_us = 0xFFFFFFFFu;
    CHECK(gs_sched_build(&c, &o, &S) != GS_OK);
    gs_cfg_default(&c); c.frame_period_us = 0xFFFFFFFFu;
    CHECK(gs_sched_build(&c, &o, &S) != GS_OK);

    /* burst with a single exposure ignores frame_period */
    gs_cfg_default(&c); c.burst_exposures = 1; c.frame_period_us = 0;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);

    /* options */
    gs_cfg_default(&c); o.lockout_us = GS_MAX_LOCKOUT_US + 1;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_BAD_VALUE);
    gs_opts_default(&o); o.vdiv_x1000 = 0;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_BAD_VALUE);
    gs_opts_default(&o); o.cam_active_high = 2;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_BAD_VALUE);
}

static void test_duty_pps_boundaries(void)
{
    gs_strobe_cfg_t c; gs_opts_t o;
    gs_opts_default(&o);
    /* free-run, 10 frames per 100 ms window: duty = 10*n*w / 100000 */
    gs_cfg_default(&c);
    c.mode = GS_MODE_FREE_RUN; c.frame_period_us = 10000; c.strobe_interval_us = 300;
    c.pulses_per_exposure = 20;              /* 20 * 10 us per frame -> exactly 2.000 % */
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    CHECK_EQ(S.duty_ppm_worst, 20000);
    c.pulses_per_exposure = 21;              /* 2.1 % -> rejected, not clipped */
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_LIMIT_DUTY);
    CHECK_EQ(S.err_detail, 21000);

    /* pulses per second boundary: n=1, w=5, period 334 us -> 2995, 333 us -> 3004 */
    gs_cfg_default(&c);
    c.mode = GS_MODE_FREE_RUN; c.strobe_width_us = 5; c.pulses_per_exposure = 1;
    c.first_pulse_delay_us = 50; c.exposure_tail_us = 50;
    c.frame_period_us = 334;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    CHECK_EQ(S.pps_worst, 2995);
    c.frame_period_us = 333;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_LIMIT_PPS);
    CHECK_EQ(S.err_detail, 3004);

    /* burst: the biggest burst (8 x 32 pulses) */
    gs_cfg_default(&c);
    c.burst_exposures = 8; c.pulses_per_exposure = 32; c.strobe_interval_us = 100;
    c.strobe_width_us = 20; c.frame_period_us = 3320; o.lockout_us = 0;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_LIMIT_DUTY);   /* 256 x 20 us in one window */
    o.lockout_us = GS_MAX_LOCKOUT_US;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_ERR_LIMIT_DUTY);   /* even one burst alone is too much */
    c.strobe_width_us = 5; c.strobe_interval_us = 10; c.frame_period_us = 600;
    o.lockout_us = GS_DEFAULT_LOCKOUT_US;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    CHECK(S.n_words <= GS_MAX_WAVE_WORDS && (S.n_words & (S.n_words - 1)) == 0);
    timeline_t t; replay(&S, &t);
    CHECK_EQ(t.n_st, 256);
    CHECK_EQ(t.total, S.span_cycles);
    /* lockout 0 on a burst: the firmware raises it to span + margin and re-checks the limits */
    o.lockout_us = 0;
    int e = gs_sched_build(&c, &o, &S);
    CHECK(e == GS_OK || e == GS_ERR_LIMIT_DUTY || e == GS_ERR_LIMIT_PPS);
    gs_cfg_default(&c);
    gs_opts_default(&o); o.lockout_us = 0;
    CHECK_EQ(gs_sched_build(&c, &o, &S), GS_OK);
    CHECK(S.lockout_eff_us >= S.span_us + GS_LOCKOUT_MARGIN_US);
    CHECK(S.lockout_eff_us < 25000);
}

static void test_window_bruteforce(void)
{
    for (int iter = 0; iter < 300; iter++) {
        uint32_t n = 1 + gs_rand() % 6;
        uint32_t period = 50 + gs_rand() % 400;
        uint32_t st[8], prev = gs_rand() % 10;
        for (uint32_t i = 0; i < n; i++) { st[i] = prev; prev += 1 + gs_rand() % ((period - 1) / n + 1); }
        if (st[n - 1] >= period) continue;
        uint32_t window = 1 + gs_rand() % 3000;
        uint32_t got = gs_max_pulses_in_window(st, n, period, window);
        /* brute force: slide over every integer start */
        uint32_t best = 0;
        for (uint32_t s = 0; s < period; s++) {
            uint32_t cnt = 0;
            for (uint32_t k = 0; k < 3000 / period + 4; k++)
                for (uint32_t j = 0; j < n; j++) {
                    uint64_t a = (uint64_t)k * period + st[j];
                    if (a >= s && a < s + window) cnt++;
                }
            if (cnt > best) best = cnt;
        }
        CHECK_EQ(got, best);
    }
}

static void test_wave_word(void)
{
    uint32_t w = gs_wave_word(true, true, true, 125);
    CHECK_EQ(w & 7, 7);
    CHECK_EQ(gs_wave_word_cycles(w), 125);
    CHECK_EQ(gs_wave_word_cycles(gs_wave_word(false, false, false, 6)), 6);
    CHECK_EQ(gs_duty_ppm(1, 20, 100000), 200);
    CHECK_EQ(gs_duty_ppm(1, 1, 3000000), 1);   /* rounds up */
}

int main(void)
{
    test_defaults();
    test_trigger_delay_and_polarity_independent();
    test_free_run();
    test_limits();
    test_duty_pps_boundaries();
    test_window_bruteforce();
    test_wave_word();
    TEST_MAIN_END();
}
