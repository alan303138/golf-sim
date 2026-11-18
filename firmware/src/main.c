/* Golf-sim timing MCU firmware: USB CDC command link + strobe/camera timing engine. */
#include <stdio.h>

#include "gs_core.h"
#include "gs_pins.h"
#include "hardware/watchdog.h"
#include "hw.h"
#include "pico/stdlib.h"

static gs_core_t g_core;
static gs_deframer_t g_deframer;

int main(void)
{
    int wdt_reset = watchdog_caused_reboot();
    gs_hw_init();                 /* pins safe before anything else (strobe low) */
    stdio_init_all();             /* USB CDC (no UART) */
    gs_deframer_init(&g_deframer);
    gs_core_init(&g_core, gs_hw_hal(), NULL, wdt_reset);
    watchdog_enable(1000, true);  /* hardware watchdog: reset if the main loop stalls for 1 s */

    for (;;) {
        watchdog_update();
        int ch;
        int budget = 256;
        while (budget-- > 0 && (ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            gs_frame_t f;
            if (gs_deframer_push(&g_deframer, (uint8_t)ch, &f)) gs_core_rx_frame(&g_core, &f);
        }
        gs_core_set_rx_bad(&g_core, g_deframer.bad_frames);
        gs_hw_poll(&g_core);
        gs_core_tick(&g_core);
    }
}
