/*
 * Runtime sliding-window monitor of the *measured* strobe pulses (backstop
 * behind the config-time worst-case analysis). Bins of 1 ms; the window is
 * GS_DUTY_WINDOW_US for the duty limit and GS_PPS_WINDOW_US for pulses/s.
 * Portable C, O(1) per pulse (called from the strobe-edge interrupt). Times are the
 * low 32 bits of the microsecond clock; the window history restarts when that wraps (71 min).
 */
#ifndef GS_LIMITER_H
#define GS_LIMITER_H

#include <stdint.h>

#include "gs_limits.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GS_LIM_BIN_US 1000u
#define GS_LIM_NBINS  (GS_PPS_WINDOW_US / GS_LIM_BIN_US)   /* covers the longer window */
#define GS_LIM_DUTY_BINS (GS_DUTY_WINDOW_US / GS_LIM_BIN_US + 1u) /* N+1 bins always cover N */

typedef struct {
    uint32_t on_us[GS_LIM_NBINS];
    uint16_t pulses[GS_LIM_NBINS];
    uint32_t cur_bin;       /* absolute bin index of the newest bin (wraps with the 32-bit clock) */
    uint32_t cur_idx;       /* its slot in the ring */
    uint32_t sum_on;        /* on-time in the newest GS_LIM_DUTY_BINS bins */
    uint32_t sum_pulses;    /* pulses in all GS_LIM_NBINS bins */
    int started;
    uint32_t total_pulses;
    uint32_t max_width_us;
} gs_limiter_t;

enum { GS_LIM_OK = 0, GS_LIM_DUTY = 1, GS_LIM_PPS = 2 };

void gs_limiter_init(gs_limiter_t *l);
/* Record one finished pulse that started at t_start_us and lasted width_us.
 * Returns GS_LIM_OK or the violated limit. Tolerance: 10 % + one bin, because
 * bins are coarser than the exact analysis done at configuration time. */
int gs_limiter_pulse(gs_limiter_t *l, uint32_t t_start_us32, uint32_t width_us);
/* Current duty (ppm) over the duty window ending at now_us. */
uint32_t gs_limiter_duty_ppm(const gs_limiter_t *l, uint32_t now_us32);

#ifdef __cplusplus
}
#endif

#endif
