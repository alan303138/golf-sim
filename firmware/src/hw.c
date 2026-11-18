/*
 * RP2040 hardware layer.
 *
 *  - SM0 (gs_wave) plays the PIO word stream; a pair of cross-chained DMA channels reads the
 *    word ring (hardware read-ring wrap) endlessly into the SM0 TX FIFO, so no CPU is
 *    involved in generating CAM_TRIG / STROBE edges.
 *  - SM1 (gs_trig) detects the TRIG_IN edge, starts SM0 (burst mode) and interrupts the CPU
 *    to take a timestamp. It enforces the lockout.
 *  - Strobe watchdog: GPIO edge interrupts on the STROBE pad (rising arms a hardware alarm
 *    at max width + slack, falling cancels it); an expired alarm with the pad still high
 *    forces the pin low through the GPIO output override and stops the PIO.
 *  - Interlock: STROBE_EN falling edge forces STROBE low immediately (output override).
 *  - Runtime limiter: the measured pulses feed gs_limiter; a violation trips like the watchdog.
 */
#include "hw.h"

#include <string.h>

#include "gs_limiter.h"
#include "gs_pins.h"
#include "gs_wave.pio.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/timer.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#define SM_WAVE 0u
#define SM_TRIG 1u
#define ALARM_NUM 2u
#define IRQ_START_TOKEN 4u
#define OVERRIDE_NORMAL 0u
#define OVERRIDE_INVERT 1u
#define OVERRIDE_LOW 2u
#define OVERRIDE_HIGH 3u
#define FORCE_INTERLOCK 1u
#define FORCE_TRIP 2u
#define EDGE_LOW_BIT 0x4u
#define EDGE_HIGH_BIT 0x8u
#define EDGE_MASK_PINS_3_4 ((0xCu << 12) | (0xCu << 16))
#define FIRE_TIMEOUT_US 5000u
#define INTERLOCK_DEBOUNCE_US 20000u
#define EVQ_SIZE 16u

static PIO const g_pio = pio0;
static uint g_off_wave, g_off_trig;
static int g_dma[2];
static uint32_t g_ring[GS_MAX_WAVE_WORDS] __attribute__((aligned(GS_MAX_WAVE_WORDS * 4u)));
static uint32_t g_ring_words;

static gs_opts_t g_opts;
static uint8_t g_mode;
static uint32_t g_lockout_cycles;
static uint32_t g_cfg_width_us = 10;

static gs_limiter_t g_lim;
static uint32_t g_rise_t;
static volatile uint8_t g_force_mask;
static volatile uint32_t g_trip;
static volatile uint8_t g_il_changed;
static volatile uint32_t g_bursts_done;
static volatile uint8_t g_soft_pending;
static uint32_t g_soft_t0;

typedef struct { uint32_t t_lo; uint8_t soft, inhibited; } ev_t;
static ev_t g_evq[EVQ_SIZE];
static volatile uint32_t g_evq_head, g_evq_tail;
static volatile uint8_t g_evq_overflow;

static uint32_t g_rail_acc;     /* ADC raw * 16, IIR filtered */
static uint32_t g_il_high_since;
static uint32_t g_trig_led_until;
static uint32_t g_burst_reported;

/* ---------------------------------------------------------------- pad control */
static inline void set_outover(uint pin, uint32_t v)
{
    hw_write_masked(&io_bank0_hw->io[pin].ctrl, v << IO_BANK0_GPIO0_CTRL_OUTOVER_LSB,
                    IO_BANK0_GPIO0_CTRL_OUTOVER_BITS);
}

static inline void set_inover(uint pin, uint32_t v)
{
    hw_write_masked(&io_bank0_hw->io[pin].ctrl, v << IO_BANK0_GPIO0_CTRL_INOVER_LSB,
                    IO_BANK0_GPIO0_CTRL_INOVER_BITS);
}

static inline void apply_force(void)
{
    set_outover(GS_PIN_STROBE, g_force_mask ? OVERRIDE_LOW : OVERRIDE_NORMAL);
}

static inline uint32_t trig_normal_inover(void)
{
    /* the PIO always sees 1 = asserted: invert the pad for falling-edge triggers */
    return g_opts.trig_rising ? OVERRIDE_NORMAL : OVERRIDE_INVERT;
}

static void __not_in_flash_func(trip)(uint32_t fault)
{
    g_force_mask |= FORCE_TRIP;
    apply_force();
    hw_clear_bits(&g_pio->ctrl, (1u << SM_WAVE) | (1u << SM_TRIG));
    g_trip |= fault;
}

/* ------------------------------------------------------------------ interrupts */
static void __not_in_flash_func(pio_irq0_handler)(void)
{
    uint32_t t = timer_hw->timerawl;               /* first thing: timestamp */
    uint32_t flags = g_pio->irq;
    if (flags & 1u) {
        g_pio->irq = 1u;
        uint32_t h = g_evq_head;
        if (h - g_evq_tail < EVQ_SIZE) {
            ev_t *e = &g_evq[h % EVQ_SIZE];
            e->t_lo = t;
            e->soft = g_soft_pending;
            e->inhibited = g_force_mask ? 1u : 0u;
            __dmb();
            g_evq_head = h + 1;
        } else {
            g_evq_overflow = 1;
        }
        if (g_soft_pending) {
            g_soft_pending = 0;
            set_inover(GS_PIN_TRIG_IN, trig_normal_inover());
        }
    }
    if (flags & 2u) {
        g_pio->irq = 2u;
        g_bursts_done++;
    }
}

static void __not_in_flash_func(gpio_irq_handler)(void)
{
    uint32_t t = timer_hw->timerawl;
    uint32_t ints = io_bank0_hw->proc0_irq_ctrl.ints[0];
    io_bank0_hw->intr[0] = ints & EDGE_MASK_PINS_3_4;   /* acknowledge edge events */

    if (ints & (0xCu << 16)) {                           /* STROBE_EN changed */
        if (!(sio_hw->gpio_in & (1u << GS_PIN_STROBE_EN))) {
            g_force_mask |= FORCE_INTERLOCK;
            apply_force();
        }
        g_il_changed = 1;
    }
    uint32_t se = (ints >> 12) & 0xFu;                   /* STROBE pad events */
    if (se & EDGE_HIGH_BIT) {
        g_rise_t = t;
        timer_hw->alarm[ALARM_NUM] = t + GS_MAX_STROBE_WIDTH_US + GS_WATCHDOG_SLACK_US;
        hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    }
    if (se & EDGE_LOW_BIT) {
        hw_clear_bits(&timer_hw->inte, 1u << ALARM_NUM);
        timer_hw->armed = 1u << ALARM_NUM;
        uint32_t w = (se & EDGE_HIGH_BIT) ? g_cfg_width_us : (t - g_rise_t);
        int r = gs_limiter_pulse(&g_lim, t, w);
        if (r == GS_LIM_DUTY) trip(GS_FAULT_DUTY_RUNTIME);
        else if (r == GS_LIM_PPS) trip(GS_FAULT_PPS_RUNTIME);
    }
}

static void __not_in_flash_func(alarm_irq_handler)(void)
{
    timer_hw->intr = 1u << ALARM_NUM;
    hw_clear_bits(&timer_hw->inte, 1u << ALARM_NUM);
    if (sio_hw->gpio_in & (1u << GS_PIN_STROBE)) trip(GS_FAULT_STROBE_OVERWIDTH);
}

/* ------------------------------------------------------------------------ init */
void gs_hw_init(void)
{
    /* Safe levels first: strobe low as a plain output, camera line idle (high = active-low default). */
    gpio_init(GS_PIN_STROBE);
    gpio_put(GS_PIN_STROBE, 0);
    gpio_set_dir(GS_PIN_STROBE, GPIO_OUT);
    gpio_init(GS_PIN_CAM_TRIG);
    gpio_put(GS_PIN_CAM_TRIG, GS_DEFAULT_CAM_ACTIVE_HIGH ? 0 : 1);
    gpio_set_dir(GS_PIN_CAM_TRIG, GPIO_OUT);

    gpio_init(GS_PIN_STROBE_EN);
    gpio_set_dir(GS_PIN_STROBE_EN, GPIO_IN);
    gpio_pull_down(GS_PIN_STROBE_EN);           /* floating = interlock open */
    gpio_init(GS_PIN_TRIG_IN);
    gpio_set_dir(GS_PIN_TRIG_IN, GPIO_IN);
    gpio_init(GS_PIN_STATUS_LED);
    gpio_set_dir(GS_PIN_STATUS_LED, GPIO_OUT);
    gpio_init(GS_PIN_BOARD_LED);
    gpio_set_dir(GS_PIN_BOARD_LED, GPIO_OUT);

    adc_init();
    adc_gpio_init(GS_PIN_RAIL_SENSE);
    adc_select_input(GS_ADC_RAIL_INPUT);
    g_rail_acc = (uint32_t)adc_read() * 16u;

    gs_limiter_init(&g_lim);
    gs_opts_default(&g_opts);

    /* PIO programs */
    g_off_wave = pio_add_program(g_pio, &gs_wave_program);
    g_off_trig = pio_add_program(g_pio, &gs_trig_program);
    pio_sm_claim(g_pio, SM_WAVE);
    pio_sm_claim(g_pio, SM_TRIG);
    g_dma[0] = dma_claim_unused_channel(true);
    g_dma[1] = dma_claim_unused_channel(true);

    /* Output pins: set PIO direction and value 0 before handing the pads to the PIO. */
    pio_sm_set_consecutive_pindirs(g_pio, SM_WAVE, GS_PIN_CAM_TRIG, 2, true);
    pio_sm_config c = gs_wave_program_get_default_config(g_off_wave);
    sm_config_set_set_pins(&c, GS_PIN_CAM_TRIG, 2);
    pio_sm_init(g_pio, SM_WAVE, g_off_wave + gs_wave_offset_entry_play, &c);
    pio_sm_exec(g_pio, SM_WAVE, pio_encode_set(pio_pins, 0));
    gs_hw_hal()->hw_apply_options(NULL, &g_opts);
    pio_gpio_init(g_pio, GS_PIN_CAM_TRIG);
    pio_gpio_init(g_pio, GS_PIN_STROBE);
    pio_gpio_init(g_pio, GS_PIN_TRIG_IN);
    gpio_set_input_enabled(GS_PIN_STROBE, true);

    /* Interrupts: highest priority for everything that is time critical. */
    pio_set_irq0_source_enabled(g_pio, pis_interrupt0, true);
    pio_set_irq0_source_enabled(g_pio, pis_interrupt1, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq0_handler);
    irq_set_priority(PIO0_IRQ_0, 0);
    irq_set_enabled(PIO0_IRQ_0, true);

    hardware_alarm_claim(ALARM_NUM);
    irq_set_exclusive_handler(TIMER_IRQ_2, alarm_irq_handler);
    irq_set_priority(TIMER_IRQ_2, 0);
    irq_set_enabled(TIMER_IRQ_2, true);

    io_bank0_hw->intr[0] = EDGE_MASK_PINS_3_4;
    hw_set_bits(&io_bank0_hw->proc0_irq_ctrl.inte[0], EDGE_MASK_PINS_3_4);
    irq_set_exclusive_handler(IO_IRQ_BANK0, gpio_irq_handler);
    irq_set_priority(IO_IRQ_BANK0, 0);
    irq_set_enabled(IO_IRQ_BANK0, true);

    if (!gpio_get(GS_PIN_STROBE_EN)) {
        g_force_mask |= FORCE_INTERLOCK;
        apply_force();
    }
}

/* ------------------------------------------------------------------ HAL ops */
static uint64_t hal_now_us(void *ctx)
{
    (void)ctx;
    return time_us_64();
}

static void hal_apply_options(void *ctx, const gs_opts_t *o)
{
    (void)ctx;
    g_opts = *o;
    /* PIO bit 1 = CAM_TRIG active: invert the pad for an active-low camera input */
    set_outover(GS_PIN_CAM_TRIG, o->cam_active_high ? OVERRIDE_NORMAL : OVERRIDE_INVERT);
    set_inover(GS_PIN_TRIG_IN, trig_normal_inover());
    if (o->trig_rising) gpio_pull_down(GS_PIN_TRIG_IN); else gpio_pull_up(GS_PIN_TRIG_IN);
}

static int hal_prepare(void *ctx, const gs_sched_t *s, const gs_opts_t *o, uint8_t mode)
{
    (void)ctx;
    if (clock_get_hz(clk_sys) != GS_CLK_HZ) return -1;
    if (s->n_words > GS_MAX_WAVE_WORDS || (s->n_words & (s->n_words - 1))) return -1;
    memcpy(g_ring, s->words, s->n_words * sizeof(uint32_t));
    g_ring_words = s->n_words;
    g_mode = mode;
    g_opts = *o;
    g_cfg_width_us = 0;   /* filled from the schedule: first strobe segment */
    for (uint32_t i = 0; i < s->n_words; i++)
        if (s->words[i] & 2u) { g_cfg_width_us = gs_wave_word_cycles(s->words[i]) / GS_CYCLES_PER_US; break; }
    uint32_t lock = s->lockout_eff_us * GS_CYCLES_PER_US;
    g_lockout_cycles = lock ? lock - 1u : 0u;
    return 0;
}

static void stop_dma(void)
{
    dma_hw->abort = (1u << g_dma[0]) | (1u << g_dma[1]);
    while (dma_hw->ch[g_dma[0]].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) tight_loop_contents();
    while (dma_hw->ch[g_dma[1]].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) tight_loop_contents();
}

static void hal_stop(void *ctx)
{
    (void)ctx;
    hw_clear_bits(&g_pio->ctrl, (1u << SM_WAVE) | (1u << SM_TRIG));
    stop_dma();
    hw_clear_bits(&timer_hw->inte, 1u << ALARM_NUM);
    timer_hw->armed = 1u << ALARM_NUM;
    pio_sm_clear_fifos(g_pio, SM_WAVE);
    pio_sm_restart(g_pio, SM_WAVE);
    pio_sm_exec(g_pio, SM_WAVE, pio_encode_set(pio_pins, 0));   /* CAM idle, STROBE low */
    g_soft_pending = 0;
    set_inover(GS_PIN_TRIG_IN, trig_normal_inover());
}

static void setup_dma_channel(int ch, int other)
{
    uint32_t ring_bits = 2;
    while ((1u << ring_bits) < g_ring_words * 4u) ring_bits++;
    dma_channel_config cfg = dma_channel_get_default_config((uint)ch);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_ring(&cfg, false, ring_bits);          /* read address wraps in the ring */
    channel_config_set_dreq(&cfg, pio_get_dreq(g_pio, SM_WAVE, true));
    channel_config_set_chain_to(&cfg, (uint)other);
    dma_channel_configure((uint)ch, &cfg, &g_pio->txf[SM_WAVE], g_ring, g_ring_words, false);
}

static int hal_start(void *ctx, uint64_t *epoch_us)
{
    (void)ctx;
    hal_stop(NULL);
    g_pio->irq = 0xFFu;
    g_force_mask &= (uint8_t)~FORCE_TRIP;
    if (!gpio_get(GS_PIN_STROBE_EN)) g_force_mask |= FORCE_INTERLOCK;
    apply_force();
    g_il_high_since = 0;
    g_trip = 0;
    g_burst_reported = g_bursts_done;

    /* SM0: entry depends on the mode */
    pio_sm_config c = gs_wave_program_get_default_config(g_off_wave);
    sm_config_set_out_pins(&c, GS_PIN_CAM_TRIG, 2);
    sm_config_set_set_pins(&c, GS_PIN_CAM_TRIG, 2);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    uint entry = g_mode == GS_MODE_BURST ? gs_wave_offset_entry_wait : gs_wave_offset_entry_play;
    pio_sm_init(g_pio, SM_WAVE, g_off_wave + entry, &c);
    pio_sm_exec(g_pio, SM_WAVE, pio_encode_set(pio_pins, 0));

    /* SM1: edge detector + lockout */
    pio_sm_config t = gs_trig_program_get_default_config(g_off_trig);
    sm_config_set_in_pins(&t, GS_PIN_TRIG_IN);
    sm_config_set_clkdiv_int_frac(&t, 1, 0);
    pio_sm_init(g_pio, SM_TRIG, g_off_trig, &t);
    pio_sm_put_blocking(g_pio, SM_TRIG, g_lockout_cycles);
    pio_sm_exec(g_pio, SM_TRIG, pio_encode_pull(false, false));
    pio_sm_exec(g_pio, SM_TRIG, pio_encode_mov(pio_y, pio_osr));

    /* DMA ping-pong ring feeder */
    setup_dma_channel(g_dma[0], g_dma[1]);
    setup_dma_channel(g_dma[1], g_dma[0]);
    dma_channel_start((uint)g_dma[0]);
    uint32_t t0 = time_us_32();
    while (pio_sm_is_tx_fifo_empty(g_pio, SM_WAVE)) {
        if (time_us_32() - t0 > 2000u) { hal_stop(NULL); return -1; }
    }
    *epoch_us = time_us_64();
    pio_enable_sm_mask_in_sync(g_pio, (1u << SM_WAVE) | (1u << SM_TRIG));
    return 0;
}

static void hal_fire(void *ctx)
{
    (void)ctx;
    g_soft_t0 = time_us_32();
    g_soft_pending = 1;
    set_inover(GS_PIN_TRIG_IN, OVERRIDE_HIGH);   /* PIO sees an asserted TRIG_IN edge */
}

static int hal_interlock_ok(void *ctx)
{
    (void)ctx;
    return gpio_get(GS_PIN_STROBE_EN);
}

static uint32_t hal_rail_raw(void *ctx)
{
    (void)ctx;
    return (g_rail_acc + 8u) / 16u;
}

static void hal_stats(void *ctx, uint32_t *pulses, uint32_t *maxw, uint32_t *duty)
{
    (void)ctx;
    *pulses = g_lim.total_pulses;
    *maxw = g_lim.max_width_us;
    *duty = gs_limiter_duty_ppm(&g_lim, time_us_32());
}

static void hal_uid(void *ctx, uint8_t uid[8])
{
    (void)ctx;
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    memcpy(uid, id.id, 8);
}

static void hal_bootloader(void *ctx)
{
    (void)ctx;
    hal_stop(NULL);
    sleep_ms(20);          /* let the RESULT frame leave the USB FIFO */
    reset_usb_boot(0, 0);
}

static void hal_tx(void *ctx, const uint8_t *d, size_t n)
{
    (void)ctx;
    if (stdio_usb_connected()) stdio_put_string((const char *)d, (int)n, false, false);
}

static const gs_hal_t g_hal = {
    hal_now_us, hal_apply_options, hal_prepare, hal_start, hal_stop, hal_fire,
    hal_interlock_ok, hal_rail_raw, hal_stats, hal_uid, hal_bootloader, hal_tx,
};

const gs_hal_t *gs_hw_hal(void) { return &g_hal; }

/* ------------------------------------------------------------------ housekeeping */
void gs_hw_poll(gs_core_t *core)
{
    /* triggers */
    while (g_evq_tail != g_evq_head) {
        ev_t e = g_evq[g_evq_tail % EVQ_SIZE];
        __dmb();
        g_evq_tail++;
        uint64_t now = time_us_64();
        uint64_t t = (now & ~0xFFFFFFFFull) | e.t_lo;
        if (t > now) t -= 0x100000000ull;
        gs_core_trigger(core, t, e.soft, e.inhibited);
        g_trig_led_until = time_us_32() + 50000u;
    }
    if (g_evq_overflow) { g_evq_overflow = 0; gs_core_fault(core, GS_FAULT_TRIG_QUEUE); }
    while (g_burst_reported != g_bursts_done) { g_burst_reported++; gs_core_burst_done(core); }

    /* software trigger that never reached the PIO (e.g. TRIG_IN stuck asserted) */
    if (g_soft_pending && time_us_32() - g_soft_t0 > FIRE_TIMEOUT_US) {
        g_soft_pending = 0;
        set_inover(GS_PIN_TRIG_IN, trig_normal_inover());
        gs_core_fault(core, GS_FAULT_FIRE_TIMEOUT);
    }

    /* faults raised in interrupts */
    uint32_t tr = g_trip;
    if (tr) {
        g_trip = 0;
        gs_core_fault(core, tr);
    }

    /* interlock with release debounce */
    bool ok = gpio_get(GS_PIN_STROBE_EN);
    uint32_t now32 = time_us_32();
    if (!ok) {
        g_il_high_since = 0;
        g_force_mask |= FORCE_INTERLOCK;
        apply_force();
        gs_core_interlock(core, 0);
    } else if (g_force_mask & FORCE_INTERLOCK) {
        if (g_il_high_since == 0) g_il_high_since = now32 | 1u;
        else if (now32 - g_il_high_since > INTERLOCK_DEBOUNCE_US) {
            g_force_mask &= (uint8_t)~FORCE_INTERLOCK;
            apply_force();
            g_il_high_since = 0;
            gs_core_interlock(core, 1);
        }
    }
    g_il_changed = 0;

    /* ADC (about every 5 ms) */
    static uint32_t adc_t;
    if (now32 - adc_t > 5000u) {
        adc_t = now32;
        adc_select_input(GS_ADC_RAIL_INPUT);
        uint32_t raw = adc_read();
        g_rail_acc = g_rail_acc - g_rail_acc / 16u + raw;
    }

    /* LEDs: board LED = heartbeat / trigger flash; status LED = armed (solid), fault (fast blink) */
    bool trig_flash = (int32_t)(g_trig_led_until - now32) > 0;
    bool beat = ((now32 / 500000u) & 1u) != 0;
    gpio_put(GS_PIN_BOARD_LED, trig_flash || beat);
    bool status;
    if (core->faults & GS_FAULT_STOPPING) status = ((now32 / 100000u) & 1u) != 0;
    else if (core->armed) status = core->interlock_ok ? true : (((now32 / 250000u) & 1u) != 0);
    else status = false;
    gpio_put(GS_PIN_STATUS_LED, status);
}
