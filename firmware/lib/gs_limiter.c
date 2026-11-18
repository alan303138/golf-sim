#include "gs_limiter.h"

#include <string.h>

/* On the RP2040 these run in the strobe-edge interrupt: keep them out of flash (XIP). */
#ifdef PICO_ON_DEVICE
#define GS_RAMFUNC __attribute__((section(".time_critical.gs_limiter"), noinline))
#else
#define GS_RAMFUNC
#endif

void gs_limiter_init(gs_limiter_t *l)
{
    memset(l, 0, sizeof(*l));
}

GS_RAMFUNC static void advance(gs_limiter_t *l, uint32_t bin)
{
    if (!l->started) {
        l->started = 1;
        l->cur_bin = bin;
        l->cur_idx = 0;
        return;
    }
    uint32_t steps = bin - l->cur_bin;
    if (steps == 0 || steps > 0x80000000u) return;     /* same bin, or clock went backwards */
    if (steps >= GS_LIM_NBINS) {
        memset(l->on_us, 0, sizeof(l->on_us));
        memset(l->pulses, 0, sizeof(l->pulses));
        l->sum_on = 0;
        l->sum_pulses = 0;
        l->cur_bin = bin;
        return;
    }
    for (uint32_t i = 0; i < steps; i++) {
        uint32_t nxt = l->cur_idx + 1;
        if (nxt == GS_LIM_NBINS) nxt = 0;
        /* bin leaving the duty window: newest-GS_LIM_DUTY_BINS+1 */
        uint32_t out = nxt + GS_LIM_NBINS - GS_LIM_DUTY_BINS;
        if (out >= GS_LIM_NBINS) out -= GS_LIM_NBINS;
        l->sum_on -= l->on_us[out];
        /* the slot being reused is the oldest bin of the pps window */
        l->sum_pulses -= l->pulses[nxt];
        l->on_us[nxt] = 0;
        l->pulses[nxt] = 0;
        l->cur_idx = nxt;
    }
    l->cur_bin = bin;
}

GS_RAMFUNC int gs_limiter_pulse(gs_limiter_t *l, uint32_t t_us, uint32_t width_us)
{
    advance(l, t_us / GS_LIM_BIN_US);
    l->on_us[l->cur_idx] += width_us;
    l->pulses[l->cur_idx]++;
    l->sum_on += width_us;
    l->sum_pulses++;
    l->total_pulses++;
    if (width_us > l->max_width_us) l->max_width_us = width_us;

    /* GS_LIM_DUTY_BINS bins always cover a full window: allow 10 % + one pulse of slack */
    uint32_t lim_on = (uint32_t)(((uint64_t)GS_MAX_DUTY_PPM * (GS_DUTY_WINDOW_US + GS_LIM_BIN_US)) / 1000000ull);
    lim_on = lim_on + lim_on / 10 + GS_MAX_STROBE_WIDTH_US;
    if (l->sum_on > lim_on) return GS_LIM_DUTY;
    uint32_t lim_pc = GS_MAX_PULSES_PER_SEC + GS_MAX_PULSES_PER_SEC / 10 + 1;
    if (l->sum_pulses > lim_pc) return GS_LIM_PPS;
    return GS_LIM_OK;
}

uint32_t gs_limiter_duty_ppm(const gs_limiter_t *l, uint32_t now_us)
{
    if (!l->started) return 0;
    const uint32_t n = GS_DUTY_WINDOW_US / GS_LIM_BIN_US;
    uint32_t age = now_us / GS_LIM_BIN_US - l->cur_bin;
    if (age > 0x80000000u || age >= n) return 0;
    /* bins still inside the window: newest (n - age) of the stored ones */
    uint64_t on = 0;
    for (uint32_t i = 0; i < n - age; i++) {
        uint32_t idx = (l->cur_idx + GS_LIM_NBINS - i) % GS_LIM_NBINS;
        on += l->on_us[idx];
    }
    return (uint32_t)(on * 1000000ull / GS_DUTY_WINDOW_US);
}
