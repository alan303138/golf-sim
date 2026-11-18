/*
 * Compile-time safety limits and timing constants. These are enforced inside
 * the firmware and can NOT be changed over the wire. Override only with -D at
 * build time, and only after re-checking the LED driver hardware limits
 * (the strobe module's hardware stuck-high cutoff acts at about 55 us).
 * Portable C, no pico-sdk dependency.
 */
#ifndef GS_LIMITS_H
#define GS_LIMITS_H

/* PIO / system clock. The schedule is computed in clock cycles; the firmware
 * refuses to arm if the real clock differs (fault GS_FAULT_CLOCK). */
#ifndef GS_CLK_HZ
#define GS_CLK_HZ 125000000u
#endif
#define GS_CYCLES_PER_US (GS_CLK_HZ / 1000000u)

/* --- strobe pulse limits --------------------------------------------- */
#ifndef GS_MAX_STROBE_WIDTH_US
#define GS_MAX_STROBE_WIDTH_US 20u        /* hard maximum single pulse */
#endif
#ifndef GS_MIN_STROBE_WIDTH_US
#define GS_MIN_STROBE_WIDTH_US 5u
#endif
#ifndef GS_MIN_STROBE_GAP_US
#define GS_MIN_STROBE_GAP_US 5u           /* min low time between two pulses */
#endif
/* Max average duty over any sliding window (parts per million). 20000 = 2 %. */
#ifndef GS_MAX_DUTY_PPM
#define GS_MAX_DUTY_PPM 20000u
#endif
#ifndef GS_DUTY_WINDOW_US
#define GS_DUTY_WINDOW_US 100000u         /* sliding window for the duty limit */
#endif
#ifndef GS_MAX_PULSES_PER_SEC
#define GS_MAX_PULSES_PER_SEC 3000u
#endif
#define GS_PPS_WINDOW_US 1000000u
/* Runtime watchdog: strobe pin high longer than max width + slack => forced low. */
#ifndef GS_WATCHDOG_SLACK_US
#define GS_WATCHDOG_SLACK_US 3u
#endif

/* --- schedule shape limits --------------------------------------------- */
#define GS_MAX_PULSES_PER_EXPOSURE 32u
#define GS_MAX_BURST_EXPOSURES     8u
#define GS_MAX_EXPOSURE_US         50000u   /* must stay <= 65535 (u16 offsets on the wire) */
#define GS_MIN_EXPOSURE_GAP_US     100u     /* idle time between two exposures */
#define GS_MAX_FRAME_PERIOD_US     1000000u
#define GS_MAX_TRIGGER_DELAY_US    1000000u
#define GS_MAX_LOCKOUT_US          10000000u
#define GS_LOCKOUT_MARGIN_US       10u      /* lockout >= burst span + margin */
#define GS_MIN_SEG_US              1u       /* min first-pulse delay / tail */

/* PIO word ring (power of two, DMA read ring) */
#define GS_MAX_WAVE_WORDS 1024u
#define GS_MIN_WAVE_WORDS 8u
#define GS_SEG_OVERHEAD_CYCLES 6u           /* PIO instructions per segment, see gs_wave.pio */
#define GS_FINAL_IDLE_CYCLES   (GS_CYCLES_PER_US)  /* idle word that ends a burst */

/* Design estimate of trigger-edge -> first PIO output change, in cycles
 * (2 input sync + wait + irq + wait + pull + out). Not measured. */
#define GS_TRIG_LATENCY_CYCLES 8u

/* Defaults (power-on options) */
#ifndef GS_DEFAULT_CAM_ACTIVE_HIGH
#define GS_DEFAULT_CAM_ACTIVE_HIGH 0u       /* 0 = active low */
#endif
#ifndef GS_DEFAULT_TRIG_RISING
#define GS_DEFAULT_TRIG_RISING 1u           /* 1 = rising edge is the shot trigger */
#endif
#ifndef GS_DEFAULT_LOCKOUT_US
#define GS_DEFAULT_LOCKOUT_US 500000u
#endif
#ifndef GS_DEFAULT_STATUS_PERIOD_MS
#define GS_DEFAULT_STATUS_PERIOD_MS 1000u
#endif
#ifndef GS_DEFAULT_VDIV_X1000
#define GS_DEFAULT_VDIV_X1000 4300u         /* R7 33k / R8 10k on the control board: 4.30 */
#endif
#ifndef GS_ADC_VREF_MV
#define GS_ADC_VREF_MV 3300u
#endif

#endif
