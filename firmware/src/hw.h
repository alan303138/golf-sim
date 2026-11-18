/* RP2040 hardware layer: PIO + DMA waveform engine, trigger capture, strobe watchdog,
 * interlock, ADC. Everything the portable core needs is exposed through gs_hal_t. */
#ifndef GS_HW_H
#define GS_HW_H

#include "gs_core.h"

/* Initialise pins, PIO, DMA, interrupts. Pins are left safe (strobe low, camera idle). */
void gs_hw_init(void);
const gs_hal_t *gs_hw_hal(void);
/* Main-loop housekeeping: forwards queued trigger events, faults and interlock
 * changes to the core, samples the ADC, drives the LEDs. */
void gs_hw_poll(gs_core_t *core);

#endif
