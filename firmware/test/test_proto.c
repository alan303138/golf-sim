#include <string.h>

#include "gs_proto.h"
#include "gs_test.h"

static void test_frame_roundtrip(void)
{
    uint8_t out[GS_PROTO_MAX_FRAME];
    const uint8_t body[] = { 1, 0, 3, 0, 0, 255 };
    size_t n = gs_frame_encode(GS_MSG_ARM, 7, body, sizeof(body), out, sizeof(out));
    CHECK(n > 0);
    CHECK_EQ(out[n - 1], 0);
    CHECK(memchr(out, 0, n - 1) == NULL);
    gs_deframer_t d;
    gs_frame_t f;
    gs_deframer_init(&d);
    int got = 0;
    for (size_t i = 0; i < n; i++) got = gs_deframer_push(&d, out[i], &f);
    CHECK_EQ(got, 1);
    CHECK_EQ(f.type, GS_MSG_ARM);
    CHECK_EQ(f.tag, 7);
    CHECK_EQ(f.body_len, sizeof(body));
    CHECK(memcmp(f.body, body, sizeof(body)) == 0);
    CHECK_EQ(d.bad_frames, 0);
    /* empty body */
    n = gs_frame_encode(GS_MSG_GET_INFO, 1, NULL, 0, out, sizeof(out));
    for (size_t i = 0; i < n; i++) got = gs_deframer_push(&d, out[i], &f);
    CHECK_EQ(got, 1);
    CHECK_EQ(f.body_len, 0);
    /* too long */
    CHECK_EQ(gs_frame_encode(1, 1, body, GS_PROTO_MAX_BODY + 1, out, sizeof(out)), 0);
    /* max body, random content with zeros */
    uint8_t big[GS_PROTO_MAX_BODY];
    for (size_t i = 0; i < sizeof(big); i++) big[i] = (uint8_t)(gs_rand() % 4 == 0 ? 0 : gs_rand());
    n = gs_frame_encode(0x42, 9, big, sizeof(big), out, sizeof(out));
    CHECK(n > 0 && n <= GS_PROTO_MAX_FRAME);
    got = 0;
    for (size_t i = 0; i < n; i++) got = gs_deframer_push(&d, out[i], &f);
    CHECK_EQ(got, 1);
    CHECK(f.body_len == sizeof(big) && memcmp(f.body, big, sizeof(big)) == 0);
}

static void test_deframer_errors(void)
{
    uint8_t out[GS_PROTO_MAX_FRAME];
    gs_deframer_t d;
    gs_frame_t f;
    gs_deframer_init(&d);
    const uint8_t body[] = { 9, 8, 7 };
    size_t n = gs_frame_encode(GS_MSG_FIRE, 3, body, 3, out, sizeof(out));
    /* every single-bit flip in a frame must be rejected (CRC-16 detects all 1-bit errors) */
    int accepted = 0;
    for (size_t i = 0; i + 1 < n; i++)
        for (int b = 0; b < 8; b++) {
            uint8_t c[GS_PROTO_MAX_FRAME];
            memcpy(c, out, n);
            c[i] ^= (uint8_t)(1u << b);
            if (c[i] == 0) continue; /* would split the frame; covered below */
            int got = 0;
            for (size_t k = 0; k < n; k++) got |= gs_deframer_push(&d, c[k], &f);
            accepted += got;
        }
    CHECK_EQ(accepted, 0);
    CHECK(d.bad_frames > 0);
    /* leading garbage + idle delimiters, then good frame, byte at a time */
    gs_deframer_init(&d);
    int got = 0;
    gs_deframer_push(&d, 0, &f);
    gs_deframer_push(&d, 0, &f);
    for (size_t k = 0; k < n; k++) got |= gs_deframer_push(&d, out[k], &f);
    CHECK_EQ(got, 1);
    /* oversize garbage without delimiter, then a good frame resyncs */
    for (int k = 0; k < 1000; k++) gs_deframer_push(&d, 0x55, &f);
    gs_deframer_push(&d, 0, &f);
    CHECK(d.bad_frames >= 1);
    got = 0;
    for (size_t k = 0; k < n; k++) got |= gs_deframer_push(&d, out[k], &f);
    CHECK_EQ(got, 1);
    /* truncated frame */
    gs_deframer_init(&d);
    for (size_t k = 0; k < n - 3; k++) gs_deframer_push(&d, out[k], &f);
    got = gs_deframer_push(&d, 0, &f);
    CHECK_EQ(got, 0);
    CHECK_EQ(d.bad_frames, 1);
}

static void test_messages(void)
{
    uint8_t b[GS_PROTO_MAX_BODY];
    gs_strobe_cfg_t c, c2;
    gs_cfg_default(&c);
    c.mode = GS_MODE_FREE_RUN; c.strobe_interval_us = 123456; c.trigger_delay_us = 0xDEADBEEFu;
    int n = gs_pack_set_config(&c, b, sizeof(b));
    CHECK_EQ(n, GS_SET_CONFIG_LEN);
    CHECK_EQ(gs_unpack_set_config(b, (size_t)n, &c2), 0);
    CHECK(memcmp(&c, &c2, sizeof(c)) == 0);
    CHECK_EQ(gs_unpack_set_config(b, (size_t)n - 1, &c2), -1);
    CHECK_EQ(gs_unpack_set_config(b, (size_t)n + 1, &c2), -1);
    CHECK_EQ(gs_pack_set_config(&c, b, 5), -1);

    gs_opts_t o, o2;
    gs_opts_default(&o);
    n = gs_pack_options(&o, b, sizeof(b));
    CHECK_EQ(n, GS_OPTIONS_LEN);
    CHECK_EQ(gs_unpack_options(b, (size_t)n, &o2), 0);
    CHECK(memcmp(&o, &o2, sizeof(o)) == 0);

    gs_status_t s, s2;
    memset(&s, 0x11, sizeof(s)); memset(&s2, 0x11, sizeof(s2));
    n = gs_pack_status(&s, b, sizeof(b));
    CHECK_EQ(n, GS_STATUS_LEN);
    CHECK_EQ(gs_unpack_status(b, (size_t)n, &s2), 0);
    CHECK(memcmp(&s, &s2, sizeof(s)) == 0);

    gs_info_t i1, i2;
    memset(&i1, 0x22, sizeof(i1)); memset(&i2, 0x22, sizeof(i2));
    n = gs_pack_info(&i1, b, sizeof(b));
    CHECK_EQ(n, GS_INFO_LEN);
    CHECK_EQ(gs_unpack_info(b, (size_t)n, &i2), 0);
    CHECK(memcmp(&i1, &i2, sizeof(i1)) == 0);

    gs_trigger_t t, t2;
    memset(&t, 0, sizeof(t));
    t.seq = 5; t.t_us = 0x0123456789ABCDEFull; t.source = 1; t.flags = 3; t.n_exp = 2; t.n_pulses = 3;
    t.exp_delta_us[0] = -16700; t.exp_delta_us[1] = 123;
    t.pulse_offset_us[0] = 50; t.pulse_offset_us[1] = 750; t.pulse_offset_us[2] = 65535;
    n = gs_pack_trigger(&t, b, sizeof(b));
    CHECK_EQ(n, 16 + 8 + 6);
    CHECK_EQ(gs_unpack_trigger(b, (size_t)n, &t2), 0);
    CHECK(memcmp(&t, &t2, sizeof(t)) == 0);
    t.n_exp = 9;
    CHECK_EQ(gs_pack_trigger(&t, b, sizeof(b)), -1);
    /* max trigger fits in a frame */
    t.n_exp = GS_MAX_BURST_EXPOSURES; t.n_pulses = GS_MAX_PULSES_PER_EXPOSURE;
    n = gs_pack_trigger(&t, b, sizeof(b));
    CHECK(n > 0 && (unsigned)n <= GS_PROTO_MAX_BODY);

    gs_result_t r = { 2, 4, 33 }, r2;
    n = gs_pack_result(&r, b, sizeof(b));
    CHECK_EQ(n, GS_RESULT_LEN);
    CHECK_EQ(gs_unpack_result(b, (size_t)n, &r2), 0);
    CHECK(r2.req_type == 2 && r2.code == 4 && r2.detail == 33);
    gs_event_t e = { 2, 0x80 }, e2;
    n = gs_pack_event(&e, b, sizeof(b));
    CHECK_EQ(n, GS_EVENT_LEN);
    CHECK_EQ(gs_unpack_event(b, (size_t)n, &e2), 0);
    CHECK(e2.code == 2 && e2.value == 0x80);

    gs_cfg_report_t cr, cr2;
    memset(&cr, 0, sizeof(cr));
    cr.cfg = c; cr.exposure_len_us = 1; cr.span_us = 2; cr.lockout_eff_us = 3; cr.duty_ppm_worst = 4;
    cr.pps_worst = 5; cr.latency_ns = 6;
    n = gs_pack_cfg_report(&cr, b, sizeof(b));
    CHECK_EQ(n, GS_CONFIG_REPORT_LEN);
    CHECK_EQ(gs_unpack_cfg_report(b, (size_t)n, &cr2), 0);
    CHECK(memcmp(&cr, &cr2, sizeof(cr)) == 0);
}

static void test_known_bytes(void)
{
    /* Fixed vector: ARM(1), tag 1 -> raw 04 01 01 | crc16(04 01 01) little-endian */
    uint8_t raw[] = { 0x04, 0x01, 0x01 };
    uint16_t crc = gs_crc16(raw, 3);
    uint8_t out[GS_PROTO_MAX_FRAME];
    const uint8_t body[] = { 1 };
    size_t n = gs_frame_encode(GS_MSG_ARM, 1, body, 1, out, sizeof(out));
    uint8_t exp_raw[] = { 0x04, 0x01, 0x01, (uint8_t)crc, (uint8_t)(crc >> 8) };
    uint8_t enc[16];
    size_t e = gs_cobs_encode(exp_raw, sizeof(exp_raw), enc, sizeof(enc));
    CHECK_EQ(n, e + 1);
    CHECK(memcmp(out, enc, e) == 0);
}

int main(void)
{
    test_frame_roundtrip();
    test_deframer_errors();
    test_messages();
    test_known_bytes();
    TEST_MAIN_END();
}
