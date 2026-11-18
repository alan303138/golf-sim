/*
 * Test-vector tool for the C <-> Python protocol cross-check (gs_mcu/tests).
 *   gs_vectors frames   print "name hexframe" for a fixed set of messages encoded by C
 *   gs_vectors decode   read hex frames on stdin; print "type tag body_hex rt" (rt = 1 if the C
 *                       unpack->pack round trip reproduces the body byte for byte), or "BAD"
 *   gs_vectors sched    read "mode pulses burst width interval first tail frame tdelay
 *                       cam_high trig_rising lockout" lines; print the computed schedule
 */
#include <stdio.h>
#include <string.h>

#include "gs_proto.h"

static void hex(const uint8_t *p, size_t n)
{
    for (size_t i = 0; i < n; i++) printf("%02x", p[i]);
}

static void emit(const char *name, uint8_t type, uint8_t tag, const uint8_t *body, int n)
{
    uint8_t out[GS_PROTO_MAX_FRAME];
    size_t len = gs_frame_encode(type, tag, body, (size_t)n, out, sizeof(out));
    printf("%s ", name);
    hex(out, len);
    printf("\n");
}

static void cmd_frames(void)
{
    uint8_t b[GS_PROTO_MAX_BODY];
    int n;
    gs_strobe_cfg_t c = { GS_MODE_FREE_RUN, 7, 3, 12, 345, 67, 89, 12345, 678901 };
    n = gs_pack_set_config(&c, b, sizeof(b));
    emit("set_config", GS_MSG_SET_CONFIG, 17, b, n);
    gs_opts_t o = { 1, 0, 250, 123456, 11000 };
    n = gs_pack_options(&o, b, sizeof(b));
    emit("set_options", GS_MSG_SET_OPTIONS, 18, b, n);
    b[0] = 1;
    emit("arm", GS_MSG_ARM, 1, b, 1);
    emit("fire", GS_MSG_FIRE, 255, NULL, 0);
    emit("get_info", GS_MSG_GET_INFO, 3, NULL, 0);
    uint8_t magic[4] = { 0xAD, 0x10, 0x07, 0xB0 };
    emit("bootloader", GS_MSG_BOOTLOADER, 4, magic, 4);
    gs_info_t i = { 1, 0, 1, 2, 3, 1, { 1, 2, 0, 4, 5, 0, 7, 8 }, 123456, 125000000, 20, 5, 20000, 100000, 3000, 32, 8, 50000 };
    n = gs_pack_info(&i, b, sizeof(b));
    emit("info", GS_MSG_INFO, 3, b, n);
    gs_status_t s = { 1, 1, 0, 1, 0x81, 99999, 11998, 1, 2, 3, 4, 5, 6, 7, 8 };
    n = gs_pack_status(&s, b, sizeof(b));
    emit("status", GS_MSG_STATUS, 0, b, n);
    gs_trigger_t t;
    memset(&t, 0, sizeof(t));
    t.seq = 77; t.t_us = 0x1122334455667788ull; t.source = 1; t.flags = 2; t.n_exp = 2; t.n_pulses = 3;
    t.exp_delta_us[0] = -1200; t.exp_delta_us[1] = 15500;
    t.pulse_offset_us[0] = 50; t.pulse_offset_us[1] = 750; t.pulse_offset_us[2] = 1450;
    n = gs_pack_trigger(&t, b, sizeof(b));
    emit("trigger", GS_MSG_TRIGGER, 0, b, n);
    gs_result_t r = { GS_MSG_SET_CONFIG, GS_ERR_LIMIT_DUTY, 21000 };
    n = gs_pack_result(&r, b, sizeof(b));
    emit("result", GS_MSG_RESULT, 17, b, n);
    gs_event_t e = { GS_EVT_FAULT, 1 };
    n = gs_pack_event(&e, b, sizeof(b));
    emit("event", GS_MSG_EVENT, 0, b, n);
    gs_cfg_report_t cr = { c, 1, 2, 3, 4, 5, 6 };
    n = gs_pack_cfg_report(&cr, b, sizeof(b));
    emit("config", GS_MSG_CONFIG, 17, b, n);
    n = gs_pack_options(&o, b, sizeof(b));
    emit("options", GS_MSG_OPTIONS, 9, b, n);
}

static int roundtrip(const gs_frame_t *f)
{
    uint8_t b[GS_PROTO_MAX_BODY];
    int n = -2;
    union { gs_strobe_cfg_t c; gs_opts_t o; gs_info_t i; gs_status_t s; gs_trigger_t t; gs_result_t r;
            gs_event_t e; gs_cfg_report_t cr; } u;
    int rc = 0;
    switch (f->type) {
    case GS_MSG_SET_CONFIG: rc = gs_unpack_set_config(f->body, f->body_len, &u.c); if (!rc) n = gs_pack_set_config(&u.c, b, sizeof(b)); break;
    case GS_MSG_SET_OPTIONS:
    case GS_MSG_OPTIONS: rc = gs_unpack_options(f->body, f->body_len, &u.o); if (!rc) n = gs_pack_options(&u.o, b, sizeof(b)); break;
    case GS_MSG_INFO: rc = gs_unpack_info(f->body, f->body_len, &u.i); if (!rc) n = gs_pack_info(&u.i, b, sizeof(b)); break;
    case GS_MSG_STATUS: rc = gs_unpack_status(f->body, f->body_len, &u.s); if (!rc) n = gs_pack_status(&u.s, b, sizeof(b)); break;
    case GS_MSG_TRIGGER: rc = gs_unpack_trigger(f->body, f->body_len, &u.t); if (!rc) n = gs_pack_trigger(&u.t, b, sizeof(b)); break;
    case GS_MSG_RESULT: rc = gs_unpack_result(f->body, f->body_len, &u.r); if (!rc) n = gs_pack_result(&u.r, b, sizeof(b)); break;
    case GS_MSG_EVENT: rc = gs_unpack_event(f->body, f->body_len, &u.e); if (!rc) n = gs_pack_event(&u.e, b, sizeof(b)); break;
    case GS_MSG_CONFIG: rc = gs_unpack_cfg_report(f->body, f->body_len, &u.cr); if (!rc) n = gs_pack_cfg_report(&u.cr, b, sizeof(b)); break;
    default: return 1; /* messages without a typed body: nothing to round-trip */
    }
    if (rc) return 0;
    return n == f->body_len && memcmp(b, f->body, (size_t)n) == 0;
}

static void cmd_decode(void)
{
    char line[1024];
    while (fgets(line, sizeof(line), stdin)) {
        gs_deframer_t d;
        gs_frame_t f;
        gs_deframer_init(&d);
        int ok = 0;
        size_t len = strlen(line);
        for (size_t i = 0; i + 1 < len; i += 2) {
            unsigned v;
            if (sscanf(&line[i], "%2x", &v) != 1) break;
            ok |= gs_deframer_push(&d, (uint8_t)v, &f);
        }
        if (!ok) { printf("BAD\n"); continue; }
        printf("%u %u ", f.type, f.tag);
        hex(f.body, f.body_len);
        printf(" %d\n", roundtrip(&f));
    }
}

static void cmd_sched(void)
{
    static gs_sched_t s;
    unsigned v[12];
    char line[256];
    while (fgets(line, sizeof(line), stdin)) {
        if (sscanf(line, "%u %u %u %u %u %u %u %u %u %u %u %u", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5],
                   &v[6], &v[7], &v[8], &v[9], &v[10], &v[11]) != 12) continue;
        gs_strobe_cfg_t c = { (uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2], v[3], v[4], v[5], v[6], v[7], v[8] };
        gs_opts_t o = { (uint8_t)v[9], (uint8_t)v[10], 1000, v[11], 4300 };
        int e = gs_sched_build(&c, &o, &s);
        if (e) { printf("ERR %d %u\n", e, s.err_detail); continue; }
        printf("OK %u %u %u %u %u %u %u %llu | ", s.n_words, s.n_real, s.exposure_len_us, s.span_us,
               s.lockout_eff_us, s.duty_ppm_worst, s.pps_worst, (unsigned long long)s.span_cycles);
        for (unsigned i = 0; i < s.n_exposures; i++) printf("%u ", s.exposure_start_rel_us[i]);
        printf("| ");
        for (unsigned i = 0; i < s.n_pulses; i++) printf("%u ", s.pulse_offsets_us[i]);
        printf("| ");
        for (unsigned i = 0; i < s.n_words; i++) printf("%08x", s.words[i]);
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) return 2;
    if (!strcmp(argv[1], "frames")) cmd_frames();
    else if (!strcmp(argv[1], "decode")) cmd_decode();
    else if (!strcmp(argv[1], "sched")) cmd_sched();
    else return 2;
    return 0;
}
