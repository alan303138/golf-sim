#include <string.h>

#include "gs_core.h"
#include "gs_test.h"

typedef struct {
    uint64_t now;
    int stops, starts, prepares, fires, boots, applies, interlock;
    uint32_t rail_raw;
    uint8_t prep_mode;
    uint8_t txbuf[8192];
    size_t txlen;
} fake_t;

static uint64_t f_now(void *c) { return ((fake_t *)c)->now; }
static void f_apply(void *c, const gs_opts_t *o) { (void)o; ((fake_t *)c)->applies++; }
static int f_prepare(void *c, const gs_sched_t *s, const gs_opts_t *o, uint8_t m)
{ (void)s; (void)o; ((fake_t *)c)->prepares++; ((fake_t *)c)->prep_mode = m; return 0; }
static int f_start(void *c, uint64_t *e) { fake_t *f = c; f->starts++; *e = f->now; return 0; }
static void f_stop(void *c) { ((fake_t *)c)->stops++; }
static void f_fire(void *c) { ((fake_t *)c)->fires++; }
static int f_il(void *c) { return ((fake_t *)c)->interlock; }
static uint32_t f_rail(void *c) { return ((fake_t *)c)->rail_raw; }
static void f_stats(void *c, uint32_t *p, uint32_t *m, uint32_t *d) { (void)c; *p = 42; *m = 11; *d = 777; }
static void f_uid(void *c, uint8_t u[8]) { (void)c; for (int i = 0; i < 8; i++) u[i] = (uint8_t)(0xA0 + i); }
static void f_boot(void *c) { ((fake_t *)c)->boots++; }
static void f_tx(void *c, const uint8_t *d, size_t n)
{ fake_t *f = c; if (f->txlen + n <= sizeof(f->txbuf)) { memcpy(f->txbuf + f->txlen, d, n); f->txlen += n; } }

static const gs_hal_t HAL = { f_now, f_apply, f_prepare, f_start, f_stop, f_fire, f_il, f_rail, f_stats, f_uid, f_boot, f_tx };

static fake_t F;
static gs_core_t C;

static void setup(void)
{
    memset(&F, 0, sizeof(F));
    F.interlock = 1;
    F.now = 1000000;
    gs_core_init(&C, &HAL, &F, 0);
}

/* parse all frames the core transmitted since the last call */
static int g_nframes;
static gs_frame_t g_frames[32];
static void collect(void)
{
    gs_deframer_t d;
    gs_deframer_init(&d);
    g_nframes = 0;
    for (size_t i = 0; i < F.txlen; i++)
        if (gs_deframer_push(&d, F.txbuf[i], &g_frames[g_nframes]) && g_nframes < 31) g_nframes++;
    F.txlen = 0;
    CHECK_EQ(d.bad_frames, 0);
}

static void send_cmd(uint8_t type, uint8_t tag, const uint8_t *body, size_t n)
{
    gs_frame_t f;
    f.type = type; f.tag = tag; f.body_len = (uint16_t)n;
    if (n) memcpy(f.body, body, n);
    gs_core_rx_frame(&C, &f);
    collect();
}

static gs_result_t result0(void)
{
    gs_result_t r = { 0xFF, 0xFF, 0 };
    CHECK_EQ(g_nframes, 1);
    CHECK_EQ(g_frames[0].type, GS_MSG_RESULT);
    CHECK_EQ(gs_unpack_result(g_frames[0].body, g_frames[0].body_len, &r), 0);
    return r;
}

static void arm(int on)
{
    uint8_t b = (uint8_t)on;
    send_cmd(GS_MSG_ARM, 1, &b, 1);
}

static void test_boot_and_info(void)
{
    setup();
    collect();
    CHECK_EQ(g_nframes, 1);
    CHECK_EQ(g_frames[0].type, GS_MSG_EVENT);
    CHECK_EQ(g_frames[0].tag, 0);
    CHECK_EQ(F.applies, 1);
    send_cmd(GS_MSG_GET_INFO, 9, NULL, 0);
    CHECK_EQ(g_nframes, 1);
    CHECK_EQ(g_frames[0].type, GS_MSG_INFO);
    CHECK_EQ(g_frames[0].tag, 9);
    gs_info_t i;
    CHECK_EQ(gs_unpack_info(g_frames[0].body, g_frames[0].body_len, &i), 0);
    CHECK_EQ(i.max_width_us, 20);
    CHECK_EQ(i.proto_major, 1);
    CHECK_EQ(i.uid[7], 0xA7);
    CHECK_EQ(i.uptime_ms, 1000);
    send_cmd(0x77, 4, NULL, 0);
    gs_result_t r = result0();
    CHECK_EQ(r.code, GS_ERR_UNKNOWN_CMD);
    CHECK_EQ(g_frames[0].tag, 4);
}

static void test_configure_and_reject(void)
{
    setup();
    collect();
    gs_strobe_cfg_t c;
    gs_cfg_default(&c);
    uint8_t b[64];
    c.strobe_width_us = 30;   /* above the compile-time maximum */
    int n = gs_pack_set_config(&c, b, sizeof(b));
    send_cmd(GS_MSG_SET_CONFIG, 2, b, (size_t)n);
    gs_result_t r = result0();
    CHECK_EQ(r.code, GS_ERR_LIMIT_WIDTH);
    CHECK_EQ(r.detail, 30);
    CHECK_EQ(C.cfg.strobe_width_us, 10);            /* old config untouched */
    c.strobe_width_us = 20; c.pulses_per_exposure = 3;
    n = gs_pack_set_config(&c, b, sizeof(b));
    send_cmd(GS_MSG_SET_CONFIG, 3, b, (size_t)n);
    CHECK_EQ(g_nframes, 1);
    CHECK_EQ(g_frames[0].type, GS_MSG_CONFIG);
    gs_cfg_report_t rep;
    CHECK_EQ(gs_unpack_cfg_report(g_frames[0].body, g_frames[0].body_len, &rep), 0);
    CHECK_EQ(rep.cfg.pulses_per_exposure, 3);
    CHECK_EQ(rep.exposure_len_us, 50 + 2 * 700 + 20 + 50);
    send_cmd(GS_MSG_SET_CONFIG, 4, b, 5);
    CHECK_EQ(result0().code, GS_ERR_BAD_LENGTH);
    /* duty rejection reaches the host with the computed duty */
    gs_cfg_default(&c);
    c.mode = GS_MODE_FREE_RUN; c.frame_period_us = 10000; c.strobe_interval_us = 300; c.pulses_per_exposure = 25;
    n = gs_pack_set_config(&c, b, sizeof(b));
    send_cmd(GS_MSG_SET_CONFIG, 5, b, (size_t)n);
    r = result0();
    CHECK_EQ(r.code, GS_ERR_LIMIT_DUTY);
    CHECK_EQ(r.detail, 25000);
    /* options: lockout shorter than the burst is lifted, bad values rejected */
    gs_opts_t o;
    gs_opts_default(&o);
    o.cam_active_high = 1; o.trig_rising = 0; o.lockout_us = 0;
    n = gs_pack_options(&o, b, sizeof(b));
    send_cmd(GS_MSG_SET_OPTIONS, 6, b, (size_t)n);
    CHECK_EQ(result0().code, GS_OK);
    CHECK_EQ(F.applies, 2);
    CHECK_EQ(C.opts.cam_active_high, 1);
    CHECK(C.sched.lockout_eff_us >= C.sched.span_us + 10);
    send_cmd(GS_MSG_GET_OPTIONS, 7, NULL, 0);
    CHECK_EQ(g_frames[0].type, GS_MSG_OPTIONS);
    o.vdiv_x1000 = 0;
    n = gs_pack_options(&o, b, sizeof(b));
    send_cmd(GS_MSG_SET_OPTIONS, 8, b, (size_t)n);
    r = result0();
    CHECK_EQ(r.code, GS_ERR_BAD_VALUE);
    CHECK_EQ(r.detail, GS_FIELD_VDIV);
    CHECK_EQ(F.applies, 2);
}

static void test_arm_fire_trigger(void)
{
    setup();
    collect();
    uint8_t fb[1];
    send_cmd(GS_MSG_FIRE, 1, fb, 0);
    CHECK_EQ(result0().code, GS_ERR_NOT_ARMED);
    arm(1);
    CHECK_EQ(result0().code, GS_OK);
    CHECK_EQ(F.starts, 1);
    CHECK_EQ(F.prep_mode, GS_MODE_BURST);
    /* config change while armed is refused */
    uint8_t b[64];
    gs_strobe_cfg_t c;
    gs_cfg_default(&c);
    int n = gs_pack_set_config(&c, b, sizeof(b));
    send_cmd(GS_MSG_SET_CONFIG, 2, b, (size_t)n);
    CHECK_EQ(result0().code, GS_ERR_BUSY_ARMED);
    /* software fire reaches the hardware */
    send_cmd(GS_MSG_FIRE, 3, NULL, 0);
    CHECK_EQ(result0().code, GS_OK);
    CHECK_EQ(F.fires, 1);
    /* the trigger comes back from the ISR path */
    gs_core_trigger(&C, 5000000, 1, 0);
    collect();
    CHECK_EQ(g_nframes, 1);
    CHECK_EQ(g_frames[0].type, GS_MSG_TRIGGER);
    CHECK_EQ(g_frames[0].tag, 0);
    gs_trigger_t t;
    CHECK_EQ(gs_unpack_trigger(g_frames[0].body, g_frames[0].body_len, &t), 0);
    CHECK_EQ(t.seq, 1);
    CHECK_EQ(t.t_us, 5000000);
    CHECK_EQ(t.source, GS_TRIG_SRC_SOFTWARE);
    CHECK_EQ(t.n_exp, 2);
    CHECK_EQ(t.exp_delta_us[0], 0);
    CHECK_EQ(t.exp_delta_us[1], 16700);
    CHECK_EQ(t.n_pulses, 5);
    CHECK_EQ(t.pulse_offset_us[4], 2850);
    /* fire inside the lockout is refused */
    F.now = 5000000 + 1000;
    send_cmd(GS_MSG_FIRE, 4, NULL, 0);
    gs_result_t r = result0();
    CHECK_EQ(r.code, GS_ERR_LOCKOUT);
    CHECK_EQ(r.detail, C.sched.lockout_eff_us - 1000);
    F.now = 5000000 + C.sched.lockout_eff_us;
    send_cmd(GS_MSG_FIRE, 5, NULL, 0);
    CHECK_EQ(result0().code, GS_OK);
    /* hardware trigger with the interlock open: reported, no pulses claimed */
    gs_core_trigger(&C, F.now + 10, 0, 1);
    collect();
    CHECK_EQ(gs_unpack_trigger(g_frames[0].body, g_frames[0].body_len, &t), 0);
    CHECK_EQ(t.seq, 2);
    CHECK_EQ(t.source, GS_TRIG_SRC_HARDWARE);
    CHECK_EQ(t.flags & GS_TRIG_FLAG_INHIBITED, GS_TRIG_FLAG_INHIBITED);
    CHECK_EQ(t.n_pulses, 0);
    send_cmd(GS_MSG_GET_STATUS, 6, NULL, 0);
    gs_status_t s;
    CHECK_EQ(gs_unpack_status(g_frames[0].body, g_frames[0].body_len, &s), 0);
    CHECK_EQ(s.armed, 1);
    CHECK_EQ(s.trig_sw, 1);
    CHECK_EQ(s.trig_hw, 1);
    CHECK_EQ(s.trig_inhibited, 1);
    CHECK_EQ(s.pulses_total, 42);
    CHECK_EQ(s.duty_ppm_now, 777);
    /* disarm: further triggers are ignored */
    arm(0);
    CHECK_EQ(result0().code, GS_OK);
    gs_core_trigger(&C, F.now + 100, 0, 0);
    collect();
    CHECK_EQ(g_nframes, 0);
}

static void test_free_run_trigger(void)
{
    setup();
    collect();
    gs_strobe_cfg_t c;
    gs_cfg_default(&c);
    c.mode = GS_MODE_FREE_RUN;
    uint8_t b[64];
    int n = gs_pack_set_config(&c, b, sizeof(b));
    send_cmd(GS_MSG_SET_CONFIG, 2, b, (size_t)n);
    CHECK_EQ(g_frames[0].type, GS_MSG_CONFIG);
    F.now = 2000000;
    arm(1);
    CHECK_EQ(F.prep_mode, GS_MODE_FREE_RUN);
    CHECK_EQ(C.epoch_us, 2000000);
    gs_core_trigger(&C, 2000000 + 3 * 16700 + 1200, 0, 0);
    collect();
    gs_trigger_t t;
    CHECK_EQ(gs_unpack_trigger(g_frames[0].body, g_frames[0].body_len, &t), 0);
    CHECK_EQ(t.flags & GS_TRIG_FLAG_FREE_RUN, GS_TRIG_FLAG_FREE_RUN);
    CHECK_EQ(t.n_exp, 2);
    CHECK_EQ(t.exp_delta_us[0], -1200);          /* the exposure that was running */
    CHECK_EQ(t.exp_delta_us[1], 16700 - 1200);   /* the next one */
    CHECK_EQ(t.n_pulses, 5);
}

static void test_faults(void)
{
    setup();
    collect();
    arm(1);
    collect();
    int stops = F.stops;
    gs_core_fault(&C, GS_FAULT_STROBE_OVERWIDTH);
    collect();
    CHECK(F.stops > stops);
    CHECK_EQ(C.armed, 0);
    CHECK_EQ(g_nframes, 2);                 /* DISARMED + FAULT events */
    CHECK_EQ(g_frames[0].type, GS_MSG_EVENT);
    arm(1);
    gs_result_t r = result0();
    CHECK_EQ(r.code, GS_ERR_FAULT_LATCHED);
    CHECK_EQ(r.detail, GS_FAULT_STROBE_OVERWIDTH);
    send_cmd(GS_MSG_CLEAR_FAULTS, 2, NULL, 0);
    CHECK_EQ(result0().code, GS_OK);
    arm(1);
    CHECK_EQ(result0().code, GS_OK);
    /* interlock opening while armed: stays armed, reported, strobe inhibited by hardware */
    gs_core_interlock(&C, 0);
    collect();
    CHECK_EQ(g_nframes, 1);
    CHECK_EQ(C.armed, 1);
    send_cmd(GS_MSG_GET_STATUS, 3, NULL, 0);
    gs_status_t s;
    CHECK_EQ(gs_unpack_status(g_frames[0].body, g_frames[0].body_len, &s), 0);
    CHECK_EQ(s.interlock_ok, 0);
    CHECK_EQ(s.flags & GS_STATUS_FLAG_STROBE_FORCED_LOW, 1);
    CHECK(s.faults & GS_FAULT_INTERLOCK_TRIPPED);
    /* arming with the interlock open is allowed but flagged in detail */
    arm(0); collect();
    arm(1);
    r = result0();
    CHECK_EQ(r.code, GS_OK);
    CHECK_EQ(r.detail, 1);
}

static void test_bootloader_and_periodic(void)
{
    setup();
    collect();
    uint8_t b[4] = { 0xAD, 0x10, 0x07, 0xB0 };
    send_cmd(GS_MSG_BOOTLOADER, 1, b, 4);
    CHECK_EQ(result0().code, GS_OK);
    CHECK_EQ(F.boots, 1);
    setup();
    collect();
    b[0] = 0;
    send_cmd(GS_MSG_BOOTLOADER, 1, b, 4);
    CHECK_EQ(result0().code, GS_ERR_BAD_MAGIC);
    CHECK_EQ(F.boots, 0);
    /* periodic status */
    F.now += 500000; gs_core_tick(&C); collect();
    CHECK_EQ(g_nframes, 0);
    F.now += 500001; gs_core_tick(&C); collect();
    CHECK_EQ(g_nframes, 1);
    CHECK_EQ(g_frames[0].type, GS_MSG_STATUS);
    CHECK_EQ(g_frames[0].tag, 0);
    F.rail_raw = 3463;
    gs_status_t s;
    send_cmd(GS_MSG_GET_STATUS, 2, NULL, 0);
    gs_unpack_status(g_frames[0].body, g_frames[0].body_len, &s);
    CHECK(s.rail_mv > 11950 && s.rail_mv < 12050);
}

int main(void)
{
    test_boot_and_info();
    test_configure_and_reject();
    test_arm_fire_trigger();
    test_free_run_trigger();
    test_faults();
    test_bootloader_and_periodic();
    TEST_MAIN_END();
}
