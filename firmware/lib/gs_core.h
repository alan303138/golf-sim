/*
 * Device logic: command handling, configuration state, fault handling and
 * message generation. Portable C: all hardware access goes through gs_hal_t,
 * so the whole behaviour is unit-tested on the host with a fake HAL.
 * Single-threaded: call everything from the main loop (interrupts only queue
 * events for the main loop).
 */
#ifndef GS_CORE_H
#define GS_CORE_H

#include "gs_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t (*now_us)(void *ctx);
    /* Apply polarity options to the pins (safe at any time while disarmed). */
    void (*hw_apply_options)(void *ctx, const gs_opts_t *o);
    /* Load the word ring, lockout, mode into the PIO/DMA (disarmed). 0 = ok. */
    int (*hw_prepare)(void *ctx, const gs_sched_t *s, const gs_opts_t *o, uint8_t mode);
    /* Start SMs + DMA. Returns 0 and the MCU time of the first free-run frame start. */
    int (*hw_start)(void *ctx, uint64_t *epoch_us);
    /* Stop everything and put CAM_TRIG idle, STROBE low. */
    void (*hw_stop)(void *ctx);
    /* Software trigger through the same input path as TRIG_IN. */
    void (*hw_fire)(void *ctx);
    int (*hw_interlock_ok)(void *ctx);
    uint32_t (*hw_rail_raw)(void *ctx);
    void (*hw_stats)(void *ctx, uint32_t *pulses_total, uint32_t *max_pulse_us, uint32_t *duty_ppm);
    void (*hw_uid)(void *ctx, uint8_t uid[8]);
    void (*hw_bootloader)(void *ctx);
    void (*tx)(void *ctx, const uint8_t *data, size_t len);
} gs_hal_t;

typedef struct {
    gs_hal_t hal;
    void *ctx;
    gs_strobe_cfg_t cfg;
    gs_opts_t opts;
    gs_sched_t sched;
    gs_sched_t scratch;
    uint8_t armed;
    uint8_t interlock_ok;
    uint32_t faults;
    uint32_t seq;
    uint32_t trig_hw, trig_sw, trig_inhibited, bursts_done, rx_bad;
    uint64_t epoch_us;
    uint64_t last_trig_us;
    int have_trig;
    uint64_t last_status_us;
} gs_core_t;

void gs_core_init(gs_core_t *c, const gs_hal_t *hal, void *ctx, int watchdog_reset);
void gs_core_rx_frame(gs_core_t *c, const gs_frame_t *f);
/* A trigger event reached the main loop. t_us = ISR timestamp. */
void gs_core_trigger(gs_core_t *c, uint64_t t_us, int software, int inhibited);
void gs_core_burst_done(gs_core_t *c);
/* Raise latched fault bits (called from the main loop when an ISR flagged one). */
void gs_core_fault(gs_core_t *c, uint32_t bits);
void gs_core_interlock(gs_core_t *c, int ok);
void gs_core_set_rx_bad(gs_core_t *c, uint32_t n);
void gs_core_tick(gs_core_t *c);

#ifdef __cplusplus
}
#endif

#endif
