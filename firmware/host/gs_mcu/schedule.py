"""Validation, strobe-limit maths and pulse-schedule computation.

A line-for-line port of firmware/lib/gs_sched.c: the MockMcuLink uses it, and
tests/test_crosscheck.py compares it with the C implementation (including the PIO words).
The limits below are the firmware's compile-time defaults; they are NOT host-configurable
on the real device (the host can only read them from INFO).
"""
from __future__ import annotations

from dataclasses import dataclass, field

from gs_common.interfaces import StrobeConfig

from . import errors as E
from .protocol import Options

CLK_HZ = 125_000_000
CYCLES_PER_US = CLK_HZ // 1_000_000
SEG_OVERHEAD_CYCLES = 6
TRIG_LATENCY_CYCLES = 8
U32 = 0xFFFFFFFF


@dataclass(frozen=True)
class Limits:
    max_width_us: int = 20
    min_width_us: int = 5
    min_gap_us: int = 5
    max_duty_ppm: int = 20_000
    duty_window_us: int = 100_000
    max_pps: int = 3000
    pps_window_us: int = 1_000_000
    max_pulses: int = 32
    max_exposures: int = 8
    max_exposure_us: int = 50_000
    min_exposure_gap_us: int = 100
    max_frame_period_us: int = 1_000_000
    max_trigger_delay_us: int = 1_000_000
    max_lockout_us: int = 10_000_000
    lockout_margin_us: int = 10
    min_seg_us: int = 1
    max_words: int = 1024
    min_words: int = 8
    final_idle_cycles: int = CYCLES_PER_US


DEFAULT_LIMITS = Limits()


@dataclass
class Schedule:
    words: list[int]
    n_real: int
    exposure_len_us: int
    n_exposures: int
    n_pulses: int
    exposure_start_rel_us: list[int]
    pulse_offsets_us: list[int]
    span_cycles: int
    span_us: int
    period_us: int
    lockout_eff_us: int
    duty_ppm_worst: int
    pps_worst: int
    latency_ns: int
    free_run: bool = False

    @property
    def n_words(self) -> int:
        return len(self.words)


def wave_word(cam: bool, strobe: bool, last: bool, cycles: int) -> int:
    x = cycles - SEG_OVERHEAD_CYCLES if cycles >= SEG_OVERHEAD_CYCLES else 0
    return int(cam) | (int(strobe) << 1) | (int(last) << 2) | ((x << 3) & U32)


def wave_word_cycles(w: int) -> int:
    return (w >> 3) + SEG_OVERHEAD_CYCLES


def max_pulses_in_window(starts: list[int], period: int, window: int) -> int:
    """Max pulse starts inside any [s, s+window) when the pattern repeats every period."""
    if not starts or window == 0 or period == 0:
        return 0
    best = 0
    for si in starts:
        cnt = 0
        for sj in starts:
            if sj >= si:
                diff = sj - si
                if diff < window:
                    cnt += (window - 1 - diff) // period + 1
            else:
                cnt += (window - 1 + (si - sj)) // period
        best = max(best, cnt)
    return min(best, U32)


def duty_ppm(count: int, width_us: int, window_us: int) -> int:
    return min(-(-(count * width_us * 1_000_000) // window_us), U32)


def rail_mv(adc_raw12: int, vref_mv: int, vdiv_x1000: int) -> int:
    return (adc_raw12 * vref_mv * vdiv_x1000 + 4096 * 1000 // 2) // (4096 * 1000)


def _fail(code: int, detail: int):
    raise E.McuError(code, detail)


def validate_options(o: Options, lim: Limits = DEFAULT_LIMITS) -> None:
    if o.lockout_us > lim.max_lockout_us or o.lockout_us < 0:
        _fail(E.BAD_VALUE, 11)
    ratio = round(o.vdiv_ratio * 1000)
    if ratio <= 0 or ratio > 1_000_000:
        _fail(E.BAD_VALUE, 13)
    if o.status_period_ms != 0 and (o.status_period_ms < 10 or o.status_period_ms > 0xFFFF):
        _fail(E.BAD_VALUE, 14)


def build(cfg: StrobeConfig, opts: Options | None = None, lim: Limits = DEFAULT_LIMITS) -> Schedule:
    """Return the schedule, or raise McuError exactly as the firmware would answer."""
    opts = opts or Options()
    cpu = CYCLES_PER_US
    mode = {"burst": 0, "free_run": 1}.get(cfg.mode)
    if mode is None:
        _fail(E.BAD_VALUE, 1)
    for v in (cfg.strobe_width_us, cfg.strobe_interval_us, cfg.first_pulse_delay_us, cfg.exposure_tail_us,
              cfg.frame_period_us, cfg.trigger_delay_us):
        if not (0 <= v <= U32):
            _fail(E.BAD_VALUE, 0)
    width = cfg.strobe_width_us
    if width < lim.min_width_us or width > lim.max_width_us:
        _fail(E.LIMIT_WIDTH, width)
    np_ = cfg.pulses_per_exposure
    if not (1 <= np_ <= lim.max_pulses):
        _fail(E.BAD_VALUE, 2)
    if not (1 <= cfg.burst_exposures <= lim.max_exposures):
        _fail(E.BAD_VALUE, 3)
    if np_ > 1 and cfg.strobe_interval_us < width + lim.min_gap_us:
        _fail(E.LIMIT_TIMING, 5)
    if cfg.first_pulse_delay_us < lim.min_seg_us:
        _fail(E.BAD_VALUE, 6)
    if cfg.exposure_tail_us < lim.min_seg_us:
        _fail(E.BAD_VALUE, 7)
    interval = cfg.strobe_interval_us if np_ > 1 else 0
    exp_len = cfg.first_pulse_delay_us + (np_ - 1) * interval + width + cfg.exposure_tail_us
    if exp_len > lim.max_exposure_us:
        _fail(E.LIMIT_TIMING, 10)
    free_run = mode == 1
    n_exp = 1 if free_run else cfg.burst_exposures
    if free_run or cfg.burst_exposures > 1:
        if cfg.frame_period_us > lim.max_frame_period_us or cfg.frame_period_us < exp_len + lim.min_exposure_gap_us:
            _fail(E.LIMIT_TIMING, 8)
    if cfg.trigger_delay_us > lim.max_trigger_delay_us:
        _fail(E.BAD_VALUE, 9)
    validate_options(opts, lim)

    pulse_offsets = [cfg.first_pulse_delay_us + p * interval for p in range(np_)]
    starts_rel = [0 if free_run else cfg.trigger_delay_us + e * cfg.frame_period_us for e in range(n_exp)]

    words: list[int] = []

    def emit(cam, strobe, cycles):
        words.append(wave_word(cam, strobe, False, cycles))

    if not free_run and cfg.trigger_delay_us > 0:
        emit(False, False, cfg.trigger_delay_us * cpu)
    for e in range(n_exp):
        emit(True, False, cfg.first_pulse_delay_us * cpu)
        for p in range(np_):
            emit(True, True, width * cpu)
            emit(True, False, (interval - width if p + 1 < np_ else cfg.exposure_tail_us) * cpu)
        if e + 1 < n_exp:
            emit(False, False, (cfg.frame_period_us - exp_len) * cpu)
    if len(words) + 1 > lim.max_words:
        _fail(E.LIMIT_TIMING, 10)
    cycles = sum(wave_word_cycles(w) for w in words)
    n_real = len(words) + 1
    ring = lim.min_words
    while ring < n_real:
        ring <<= 1
    if ring > lim.max_words:
        _fail(E.LIMIT_TIMING, 10)
    n_pad = ring - n_real
    for _ in range(n_pad):
        emit(False, False, SEG_OVERHEAD_CYCLES)
    cycles += n_pad * SEG_OVERHEAD_CYCLES
    if free_run:
        last = cfg.frame_period_us * cpu - cycles
        words.append(wave_word(False, False, False, last))
        cycles += last
    else:
        words.append(wave_word(False, False, True, lim.final_idle_cycles))
        cycles += lim.final_idle_cycles
    span_us = -(-cycles // cpu)

    if free_run:
        lockout_eff = opts.lockout_us
        period = cfg.frame_period_us
    else:
        lockout_eff = max(opts.lockout_us, span_us + lim.lockout_margin_us)
        period = lockout_eff
    starts = [s + o for s in starts_rel for o in pulse_offsets]
    cnt_w = max_pulses_in_window(starts, period, lim.duty_window_us)
    cnt_s = max_pulses_in_window(starts, period, lim.pps_window_us)
    duty = duty_ppm(cnt_w, width, lim.duty_window_us)
    if cnt_w * width * 1_000_000 > lim.max_duty_ppm * lim.duty_window_us:
        _fail(E.LIMIT_DUTY, duty)
    if cnt_s > lim.max_pps:
        _fail(E.LIMIT_PPS, cnt_s)
    return Schedule(words=words, n_real=n_real, exposure_len_us=exp_len, n_exposures=n_exp, n_pulses=np_,
                    exposure_start_rel_us=starts_rel, pulse_offsets_us=pulse_offsets, span_cycles=cycles,
                    span_us=span_us, period_us=period, lockout_eff_us=lockout_eff, duty_ppm_worst=duty,
                    pps_worst=cnt_s, latency_ns=TRIG_LATENCY_CYCLES * 1000 // cpu, free_run=free_run)
