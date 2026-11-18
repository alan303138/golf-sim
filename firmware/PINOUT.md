# Pico pin map

Raspberry Pi Pico (RP2040, not the W: the on-board LED is GP25). Matches `hardware/electronics`
and `src/gs_pins.h`. Logic levels are 3.3 V on the Pico; the camera level shifting (1.8 V / 3.3 V
jumper) and the strobe gate driver (TC4427 class) are on the control board.

| Pin | Net | Dir | Function | Default / polarity |
|---|---|---|---|---|
| GP2 | CAM_TRIG | PIO out | Camera trigger to both IMX296 modules. Exposure = time the line is active (+ about 14 us) | Active **low** by default (idle high, exposure = low pulse). Runtime option `cam_active_high`, compile-time default `GS_DEFAULT_CAM_ACTIVE_HIGH`. Implemented with the GPIO output-invert override, so the PIO words are polarity independent. Needs an external pull-up to the idle level (the board has 10 k) because the pin is high-Z while the Pico is in reset or in the bootloader. |
| GP3 | STROBE | PIO out | Gate of the IR LED driver (TC4427-class gate driver) | Active **high**, 5..20 us. Needs an external pull-down (board has one). Forced low by the firmware when STROBE_EN is low or a safety trip occurs. |
| GP4 | STROBE_EN | input | Hardware interlock state | High = strobe allowed. Internal pull-down: floating or unplugged means "open". The board also ANDs the gate with this signal in hardware; the firmware reads it, forces GP3 low within about 1 us of a falling edge, and reports it. |
| GP5 | TRIG_IN | input (PIO) | Fast shot trigger from the comparator (acoustic or optical) | Runtime option `trig_rising` (default 1: idle low, rising edge = shot; match jumper JP3 on the board). Internal pull-down for rising, pull-up for falling. 3.3 V logic only. |
| GP6 | STATUS | out | Status LED | off: disarmed, on: armed, 4 Hz blink while armed with interlock open, 10 Hz blink on a stopping fault |
| GP25 | board LED | out | Heartbeat 1 Hz, 50 ms flash on every trigger | |
| GP28 / ADC2 | RAIL_SENSE | analog in | LED rail (capacitor bank) through the divider | 12-bit, Vref 3.3 V, `vdiv_ratio` option (default 4.30 = 33 k / 10 k). Do not exceed 3.3 V at the pin. |

GP2 and GP3 must stay adjacent (one `out pins, 2` instruction drives both in the same clock cycle).
USB: the native USB port is the only host link (CDC); UART is disabled.
