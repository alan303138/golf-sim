/* Pin map of the timing MCU (Raspberry Pi Pico, RP2040). See ../PINOUT.md. */
#ifndef GS_PINS_H
#define GS_PINS_H

#define GS_PIN_CAM_TRIG   2   /* PIO out: camera trigger (polarity from options), exposure = active time */
#define GS_PIN_STROBE     3   /* PIO out: IR strobe gate, active high (must be CAM_TRIG + 1) */
#define GS_PIN_STROBE_EN  4   /* input: hardware interlock, high = strobe allowed */
#define GS_PIN_TRIG_IN    5   /* input: shot trigger (edge polarity from options) */
#define GS_PIN_STATUS_LED 6   /* output: status LED */
#define GS_PIN_BOARD_LED  25  /* output: Pico on-board LED (not for Pico W) */
#define GS_PIN_RAIL_SENSE 28  /* ADC2: LED rail through a divider */
#define GS_ADC_RAIL_INPUT 2

#if GS_PIN_STROBE != GS_PIN_CAM_TRIG + 1
#error "PIO outputs CAM_TRIG and STROBE must be adjacent pins (out pins, 2)"
#endif

#endif
