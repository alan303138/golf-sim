# Timing-MCU wire protocol, version 1.0

Transport: USB CDC serial (Raspberry Pi Pico, USB `2e8a:000a`, product string `GolfSim-Timing`).
Baud rate is irrelevant (CDC). Binary, little-endian everywhere. The device is the single source of
truth for limits; this file is the contract implemented by `lib/gs_proto.c` (C) and
`host/gs_mcu/protocol.py` (Python). `host/tests/test_crosscheck.py` checks that both produce the
same bytes.

## 1. Framing

```
frame   = COBS(payload) 0x00
payload = type:u8  tag:u8  body[0..200]  crc16:u16le
crc16   = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0) over type|tag|body
```

* COBS is the standard variant (Cheshire & Baker); the encoded data never contains `0x00`, so `0x00`
  is the frame delimiter. Empty frames (consecutive delimiters) are ignored, so the host may send a
  leading `0x00` to resynchronise.
* A frame with bad COBS, bad CRC, payload shorter than 4 bytes or longer than 204 bytes is dropped
  silently and counted in `STATUS.rx_bad_frames`. Nothing is ever executed from a damaged frame.
* `tag`: chosen by the host (1..255) in a request and echoed in the reply; unsolicited messages
  (`TRIGGER`, `EVENT`, periodic `STATUS`) use tag 0. Replies are strictly one per request.
* Example: `ARM(1)` with tag 1 is `06 04 01 01 4c 33 00` (raw payload `04 01 01 4c 33`, CRC `0x334c`);
  `GET_INFO` tag 1 is `05 01 01 1f 3e 00`.

## 2. Messages

Host to MCU (requests):

| type | name | body | reply |
|---|---|---|---|
| 0x01 | GET_INFO | empty | INFO |
| 0x02 | SET_CONFIG | 28 bytes (2.1) | CONFIG on success, RESULT(error) otherwise |
| 0x03 | SET_OPTIONS | 12 bytes (2.2) | RESULT |
| 0x04 | ARM | `armed:u8` (0 or 1) | RESULT (`detail`=1: armed but STROBE_EN is low) |
| 0x05 | FIRE | empty | RESULT |
| 0x06 | GET_STATUS | empty | STATUS |
| 0x07 | GET_CONFIG | empty | CONFIG |
| 0x08 | BOOTLOADER | `magic:u32` = 0xB00710AD | RESULT, then the MCU reboots into the USB bootloader (BOOTSEL) |
| 0x09 | CLEAR_FAULTS | empty | RESULT |
| 0x0A | GET_OPTIONS | empty | OPTIONS |

MCU to host:

| type | name | tag | meaning |
|---|---|---|---|
| 0x80 | RESULT | echo | `req_type:u8, code:u8, detail:u32`. `code` 0 = OK, otherwise an error (section 3) |
| 0x81 | INFO | echo | firmware/limits (2.3) |
| 0x82 | CONFIG | echo | accepted configuration + derived values (2.4) |
| 0x83 | STATUS | echo or 0 | status (2.5); sent unsolicited every `status_period_ms` |
| 0x84 | TRIGGER | 0 | one per accepted shot trigger (2.6) |
| 0x85 | EVENT | 0 | `code:u8, value:u32` (section 4) |
| 0x86 | OPTIONS | echo | same layout as SET_OPTIONS |

Rules: configuration (`SET_CONFIG`, `SET_OPTIONS`) is accepted only while disarmed
(`BUSY_ARMED` otherwise) and is transactional: a rejected command leaves the old configuration in
place. Nothing is ever clipped to a limit; a configuration that would exceed a limit is rejected.
On boot the firmware holds the defaults of `gs_common.interfaces.StrobeConfig` and the default
options below; they are not stored in flash, so the host must configure after every (re)boot
(the `BOOT` event announces one).

### 2.1 SET_CONFIG body (28 bytes) = `gs_common.interfaces.StrobeConfig`

| off | type | field |
|---|---|---|
| 0 | u8 | mode: 0 burst, 1 free_run |
| 1 | u8 | pulses_per_exposure (1..32) |
| 2 | u8 | burst_exposures (1..8) |
| 3 | u8 | reserved, send 0 |
| 4 | u32 | strobe_width_us (5..20) |
| 8 | u32 | strobe_interval_us (pulse start to pulse start; at least width + 5) |
| 12 | u32 | first_pulse_delay_us (exposure start to first pulse, at least 1) |
| 16 | u32 | exposure_tail_us (last pulse end to exposure end, at least 1) |
| 20 | u32 | frame_period_us (exposure start to exposure start; at least exposure length + 100; used when burst_exposures > 1 or free_run) |
| 24 | u32 | trigger_delay_us (shot trigger to first exposure start, burst mode; at most 1 000 000) |

Exposure length `E = first_pulse_delay + (pulses-1)*interval + width + tail` (at most 50 000 us).
CAM_TRIG is held active for exactly `E`; the strobe fires at offsets `first_pulse_delay + k*interval`.

### 2.2 SET_OPTIONS / OPTIONS body (12 bytes)

| off | type | field | default |
|---|---|---|---|
| 0 | u8 | cam_active_high: 0 = CAM_TRIG active low (exposure = low pulse), 1 = active high | 0 |
| 1 | u8 | trig_rising: 1 = rising edge on TRIG_IN is the shot, 0 = falling edge | 1 |
| 2 | u16 | status_period_ms (0 = off, otherwise 10..65535) | 1000 |
| 4 | u32 | lockout_us: TRIG_IN ignored for this long after an accepted trigger (0..10 000 000) | 500 000 |
| 8 | u32 | vdiv_x1000: LED-rail divider ratio times 1000 (Vrail = Vadc * ratio) | 4300 |

In burst mode the lockout actually programmed is `max(lockout_us, burst_span_us + 10)`; it is
reported as `lockout_eff_us` in CONFIG. The strobe limits are evaluated with the shortest repetition
period that lockout allows.

### 2.3 INFO body (48 bytes)

`proto_major:u8, proto_minor:u8, fw_major:u8, fw_minor:u8, fw_patch:u8, hw_revision:u8,
uid:u8[8]` (RP2040 unique board id; the host shows it as the serial), `uptime_ms:u32, clk_hz:u32`
(125 000 000), `max_width_us:u32, min_width_us:u32, max_duty_ppm:u32, duty_window_us:u32, max_pps:u32,
max_pulses_per_exposure:u8, max_burst_exposures:u8, max_exposure_us:u32`.
These are the compile-time safety limits (defaults 20 us, 5 us, 20000 ppm, 100000 us, 3000/s, 32, 8,
50000 us) and cannot be changed over the wire.

### 2.4 CONFIG body (52 bytes)

The 28 bytes of 2.1 as accepted, then `exposure_len_us, span_us` (burst: trigger to end of the word
stream; free_run: the frame period), `lockout_eff_us, duty_ppm_worst` (worst sliding-window duty over
`duty_window_us`, rounded up), `pps_worst` (worst pulses in any 1 s window), `latency_ns` (design
estimate of trigger edge to first output change) - all u32.

### 2.5 STATUS body (48 bytes)

`armed:u8, mode:u8, interlock_ok:u8` (1 = STROBE_EN high), `flags:u8` (bit0: strobe forced low),
`faults:u32` (latched, section 5), `uptime_ms, rail_mv` (LED rail, from the ADC and `vdiv_x1000`),
`trig_hw, trig_sw` (accepted triggers), `trig_inhibited` (triggers while the strobe was inhibited),
`bursts_done` (end-of-burst interrupts), `pulses_total` (strobe pulses *measured on the pad*),
`rx_bad_frames, duty_ppm_now` (measured, last `duty_window_us`), `max_pulse_us_seen` - all u32.
The pulse width is measured with a 1 us timer in an interrupt and is accurate to about +-1 us.

### 2.6 TRIGGER body (16 + 4*n_exp + 2*n_pulses bytes)

| field | type |
|---|---|
| seq | u32 (counts accepted triggers since boot, wraps; starts at 1) |
| t_us | u64, MCU microsecond clock at the trigger event (interrupt timestamp) |
| source | u8: 0 hardware TRIG_IN, 1 software FIRE |
| flags | u8: bit0 strobe inhibited (STROBE_EN was low, `n_pulses` = 0), bit1 free_run event |
| n_exp, n_pulses | u8, u8 (at most 8 and 32) |
| exp_delta_us[n_exp] | i32: exposure start minus `t_us` |
| pulse_offset_us[n_pulses] | u16: pulse start relative to each exposure start (same for every exposure) |

* burst: `exp_delta_us[i] = trigger_delay_us + i*frame_period_us`, `n_exp = burst_exposures`.
  The host computes the absolute exposure start as `t_us + delta`.
* free_run: the event is only timestamped. `n_exp` = 2: the exposure that started at or before the
  event (delta <= 0) and the next one, derived from the MCU time at which the frame generator was
  started and `frame_period_us`.
* The deltas and offsets are not measurements: they are the exact values the PIO programs
  (computed from the configuration, one clock cycle = 8 ns resolution, all values are integer
  microseconds). `t_us` is taken in an interrupt and is late by the interrupt latency (design
  estimate 0.3 to 1.5 us), so the absolute position of the whole sequence has that uncertainty while
  the shape inside it is exact.

## 3. Error codes (RESULT.code)

| code | name | detail |
|---|---|---|
| 0 | OK | ARM: 1 = strobe inhibited (STROBE_EN low) |
| 1 | UNKNOWN_CMD | the type |
| 2 | BAD_LENGTH | received body length |
| 3 | BAD_VALUE | field id (below) |
| 4 | LIMIT_WIDTH | requested width in us (outside 5..20) |
| 5 | LIMIT_DUTY | worst-case duty in ppm (limit 20000 over 100 ms) |
| 6 | LIMIT_PPS | worst-case pulses in 1 s (limit 3000) |
| 7 | LIMIT_TIMING | field id (interval, exposure length, frame period) |
| 8 | BUSY_ARMED | |
| 9 | NOT_ARMED | |
| 10 | FAULT_LATCHED | fault mask; send CLEAR_FAULTS first |
| 11 | LOCKOUT | microseconds until a software trigger is accepted again |
| 12 | BAD_MAGIC | |
| 13 | HW | hardware layer refused (e.g. system clock is not 125 MHz) |

Field ids: 1 mode, 2 pulses, 3 burst_exposures, 4 width, 5 interval, 6 first_pulse_delay, 7 tail,
8 frame_period, 9 trigger_delay, 10 exposure length, 11 lockout, 12 polarity, 13 vdiv, 14 status_period.

## 4. Events (EVENT.code, value)

1 BOOT (1 = watchdog reset), 2 FAULT (newly raised fault bits), 3 INTERLOCK (1 = STROBE_EN high,
0 = low), 4 DISARMED (fault bits that forced a disarm).

## 5. Fault bits (STATUS.faults, latched until CLEAR_FAULTS)

| bit | name | effect |
|---|---|---|
| 0 | STROBE_OVERWIDTH | watchdog saw the strobe pad high longer than max width + 3 us; forced low, disarmed |
| 1 | DUTY_RUNTIME | measured duty above the limit (+10 % tolerance); forced low, disarmed |
| 2 | PPS_RUNTIME | measured pulses per second above the limit; forced low, disarmed |
| 3 | INTERLOCK_TRIPPED | STROBE_EN went low (informational; the strobe is forced low while it is low) |
| 4 | CLOCK | system clock is not 125 MHz (arming refused) |
| 5 | WDT_RESET | the last reset came from the hardware watchdog |
| 6 | TRIG_QUEUE | trigger events were lost (host too slow to be serviced) |
| 7 | FIRE_TIMEOUT | a software trigger did not reach the PIO within 5 ms (TRIG_IN stuck asserted?) |

Bits 0, 1, 2 and 4 refuse ARM/FIRE until cleared.

## 6. Versioning

`INFO.proto_major/minor`. The major number changes when an existing message layout changes
incompatibly; the host must refuse a different major (SerialMcuLink does). The minor number is
increased for additions (new message types, new trailing fields in a message that carries its own
length check); a host must ignore unknown message types. Firmware 1.0.0 speaks protocol 1.0.
