/*
 * Configuration validation, strobe-limit maths and pulse-schedule computation.
 * Portable C, no pico-sdk dependency.
 *
 * The schedule is a list of PIO "segment words". Each word holds the logic
 * state of the two output pins (bit0 = CAM_TRIG active, bit1 = STROBE on), a
 * "last word of burst" flag (bit2) and a 29-bit loop count x (bits 3..31).
 * The PIO program holds the state for exactly x + GS_SEG_OVERHEAD_CYCLES
 * clock cycles, so a segment of T cycles is encoded with x = T - 6.
 */
#ifndef GS_SCHED_H
#define GS_SCHED_H

#include <stdbool.h>
#include <stdint.h>

#include "gs_err.h"
#include "gs_limits.h"

#ifdef __cplusplus
extern "C" {
#endif

enum gs_mode { GS_MODE_BURST = 0, GS_MODE_FREE_RUN = 1 };

/* Mirror of gs_common.interfaces.StrobeConfig. */
typedef struct {
    uint8_t mode;
    uint8_t pulses_per_exposure;
    uint8_t burst_exposures;
    uint32_t strobe_width_us;
    uint32_t strobe_interval_us;
    uint32_t first_pulse_delay_us;
    uint32_t exposure_tail_us;
    uint32_t frame_period_us;
    uint32_t trigger_delay_us;
} gs_strobe_cfg_t;

/* Hardware options that are not part of StrobeConfig. */
typedef struct {
    uint8_t cam_active_high;     /* 0 = CAM_TRIG active low (default) */
    uint8_t trig_rising;         /* 1 = rising edge on TRIG_IN is the trigger */
    uint16_t status_period_ms;   /* 0 = no periodic STATUS */
    uint32_t lockout_us;         /* TRIG_IN lockout after an accepted trigger */
    uint32_t vdiv_x1000;         /* LED-rail divider ratio * 1000 (Vrail = Vadc * ratio) */
} gs_opts_t;

typedef struct {
    uint32_t n_words;            /* words in the DMA ring (power of two, incl. padding) */
    uint32_t n_real;             /* words before padding */
    uint32_t exposure_len_us;    /* one exposure: CAM_TRIG active time */
    uint32_t n_exposures;        /* burst: burst_exposures; free_run: 1 */
    uint32_t n_pulses;           /* strobe pulses per exposure */
    uint32_t exposure_start_rel_us[GS_MAX_BURST_EXPOSURES]; /* burst: from trigger; free_run: [0]=0 */
    uint16_t pulse_offsets_us[GS_MAX_PULSES_PER_EXPOSURE];  /* from exposure start */
    uint64_t span_cycles;        /* burst: total length of the word stream; free_run: frame period */
    uint32_t span_us;            /* span_cycles / cycles-per-us, rounded up */
    uint32_t period_us;          /* worst-case repetition period used by the limit analysis */
    uint32_t lockout_eff_us;     /* lockout actually programmed into the PIO */
    uint32_t duty_ppm_worst;     /* worst sliding-window duty (ppm, rounded up) */
    uint32_t pps_worst;          /* worst pulses in any 1 s window */
    uint32_t latency_ns;         /* design estimate trigger edge -> first output change */
    uint32_t err_detail;         /* set when validation fails */
    uint32_t words[GS_MAX_WAVE_WORDS];
} gs_sched_t;

/* Default StrobeConfig / options (match gs_common.interfaces.StrobeConfig defaults). */
void gs_cfg_default(gs_strobe_cfg_t *cfg);
void gs_opts_default(gs_opts_t *o);

/* Validate options only (ranges). Returns gs_err; *detail gets a gs_field. */
int gs_opts_validate(const gs_opts_t *o, uint32_t *detail);

/* Validate cfg+opts and compute the full schedule into *out (out->words filled).
 * Returns GS_OK or a gs_err; on error out->err_detail holds the detail and
 * nothing is clipped. */
int gs_sched_build(const gs_strobe_cfg_t *cfg, const gs_opts_t *opts, gs_sched_t *out);

/* Encode / decode one PIO word. cycles must be >= GS_SEG_OVERHEAD_CYCLES. */
uint32_t gs_wave_word(bool cam, bool strobe, bool last, uint32_t cycles);
uint32_t gs_wave_word_cycles(uint32_t word);

/* Max number of pulse starts inside any half-open window [s, s+window) when the
 * pattern start_us[0..n) (sorted, all < period) repeats every period_us. */
uint32_t gs_max_pulses_in_window(const uint32_t *start_us, uint32_t n,
                                 uint32_t period_us, uint32_t window_us);

/* ceil(count * width_us * 1e6 / window_us) */
uint32_t gs_duty_ppm(uint32_t count, uint32_t width_us, uint32_t window_us);

/* LED rail voltage from a 12-bit ADC reading. */
uint32_t gs_rail_mv(uint32_t adc_raw12, uint32_t vref_mv, uint32_t vdiv_x1000);

#ifdef __cplusplus
}
#endif

#endif
