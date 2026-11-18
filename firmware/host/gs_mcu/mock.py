"""MockMcuLink: a hardware-free McuLink with the same behaviour as the firmware core.

It uses the same schedule/limit code as the firmware port (schedule.py), enforces the same
state rules (config only while disarmed, lockout, latched faults), and produces ShotTrigger
callbacks with the exact computed exposure starts and pulse offsets. Tests can simulate
hardware triggers, faults, the interlock and the LED rail voltage.

Callbacks run synchronously in the thread that caused the trigger (fire()/simulate_trigger()).
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable

from gs_common.interfaces import McuInfo, McuLink, ShotTrigger, StrobeConfig

from . import errors as E
from . import protocol as P
from . import schedule as S
from .convert import McuShotTrigger

STOPPING_FAULTS = 0x01 | 0x02 | 0x04 | 0x10


class ManualClock:
    """Deterministic microsecond clock for tests."""

    def __init__(self, start_us: int = 1_000_000):
        self.now_us = start_us

    def __call__(self) -> int:
        return self.now_us

    def advance(self, us: int) -> None:
        self.now_us += us


class MockMcuLink(McuLink):
    def __init__(self, clock: Callable[[], int] | None = None, serial: str = "4d4f434b30303031"):
        if clock is None:
            t0 = time.monotonic()
            clock = lambda: int((time.monotonic() - t0) * 1e6) + 1_000_000  # noqa: E731
        self._clock = clock
        self._boot_us = clock()
        self._serial = serial
        self._lock = threading.RLock()
        self._open = False
        self._cbs: list[Callable[[ShotTrigger], None]] = []
        self._event_cbs: list[Callable[[str, int], None]] = []
        self.config = StrobeConfig()
        self.options = P.Options()
        self._sched = S.build(self.config, self.options)
        self.armed = False
        self.faults = 0
        self.interlock_ok = True
        self.rail_mv = 12000
        self.seq = 0
        self.trig_hw = self.trig_sw = self.trig_inhibited = 0
        self.pulses_total = 0
        self._epoch = 0
        self._last_trig: int | None = None
        self.strobe_inhibited_on_arm = False
        self.triggers: list[McuShotTrigger] = []   # every trigger emitted (handy in tests)

    # ---------------------------------------------------------------- helpers
    def _require_open(self) -> None:
        if not self._open:
            raise E.McuTimeout("link not open")

    def _emit_event(self, name: str, value: int) -> None:
        for cb in list(self._event_cbs):
            cb(name, value)

    @property
    def schedule(self) -> S.Schedule:
        return self._sched

    # ---------------------------------------------------------------- McuLink
    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    def __enter__(self) -> "MockMcuLink":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def info(self) -> McuInfo:
        self._require_open()
        return McuInfo(fw_version="1.0.0", hw_revision=1, serial=self._serial,
                       uptime_ms=(self._clock() - self._boot_us) // 1000)

    def configure(self, config: StrobeConfig) -> None:
        with self._lock:
            self._require_open()
            if self.armed:
                raise E.McuError(E.BUSY_ARMED, 0, "SET_CONFIG")
            try:
                sched = S.build(config, self.options)
            except E.McuError as exc:
                raise E.McuError(exc.code, exc.detail, "SET_CONFIG") from None
            self.config = StrobeConfig(**vars(config))
            self._sched = sched

    def set_options(self, options: P.Options) -> None:
        with self._lock:
            self._require_open()
            if self.armed:
                raise E.McuError(E.BUSY_ARMED, 0, "SET_OPTIONS")
            try:
                sched = S.build(self.config, options)
            except E.McuError as exc:
                raise E.McuError(exc.code, exc.detail, "SET_OPTIONS") from None
            self.options = P.Options(**vars(options))
            self._sched = sched

    def get_options(self) -> P.Options:
        return P.Options(**vars(self.options))

    def get_config(self) -> P.ConfigReport:
        s = self._sched
        return P.ConfigReport(self.config, s.exposure_len_us, s.span_us, s.lockout_eff_us,
                              s.duty_ppm_worst, s.pps_worst, s.latency_ns)

    def arm(self, armed: bool) -> None:
        with self._lock:
            self._require_open()
            if not armed:
                self.armed = False
                return
            if self.armed:
                return
            if self.faults & STOPPING_FAULTS:
                raise E.McuError(E.FAULT_LATCHED, self.faults, "ARM")
            self.armed = True
            self._epoch = self._clock()
            self._last_trig = None
            self.strobe_inhibited_on_arm = not self.interlock_ok

    def fire(self) -> None:
        with self._lock:
            self._require_open()
            if not self.armed:
                raise E.McuError(E.NOT_ARMED, 0, "FIRE")
            if self.faults & STOPPING_FAULTS:
                raise E.McuError(E.FAULT_LATCHED, self.faults, "FIRE")
            now = self._clock()
            if self._last_trig is not None:
                until = self._last_trig + self._sched.lockout_eff_us
                if now < until:
                    raise E.McuError(E.LOCKOUT, until - now, "FIRE")
            self._trigger(now, software=True)

    def on_trigger(self, callback: Callable[[ShotTrigger], None]) -> None:
        self._cbs.append(callback)

    def on_event(self, callback: Callable[[str, int], None]) -> None:
        self._event_cbs.append(callback)

    def status(self) -> dict[str, Any]:
        from .convert import status_to_dict
        self._require_open()
        now = self._clock()
        msg = P.StatusMsg(
            armed=self.armed, mode=P.MODE_FREE_RUN if self.config.mode == "free_run" else P.MODE_BURST,
            interlock_ok=self.interlock_ok, flags=0 if self.interlock_ok else P.STATUS_FLAG_STROBE_FORCED_LOW,
            faults=self.faults, uptime_ms=(now - self._boot_us) // 1000, rail_mv=self.rail_mv,
            trig_hw=self.trig_hw, trig_sw=self.trig_sw, trig_inhibited=self.trig_inhibited,
            bursts_done=self.trig_hw + self.trig_sw, pulses_total=self.pulses_total,
            rx_bad_frames=0, duty_ppm_now=0, max_pulse_us_seen=self.config.strobe_width_us if self.pulses_total else 0)
        return status_to_dict(msg, "1.0.0")

    def clear_faults(self) -> None:
        self.faults = 0

    def reboot_to_bootloader(self) -> None:
        self.armed = False
        self._open = False

    # --------------------------------------------------------- test controls
    def simulate_trigger(self, t_us: int | None = None) -> McuShotTrigger | None:
        """Hardware TRIG_IN edge. Returns the ShotTrigger, or None if it was ignored
        (disarmed, faulted, or inside the lockout) exactly as the PIO would ignore it."""
        with self._lock:
            self._require_open()
            if not self.armed or self.faults & STOPPING_FAULTS:
                return None
            t = self._clock() if t_us is None else t_us
            if self._last_trig is not None and t < self._last_trig + self._sched.lockout_eff_us:
                return None
            return self._trigger(t, software=False)

    def simulate_fault(self, mask: int) -> None:
        """Raise latched fault bits like the firmware does (stopping faults disarm)."""
        with self._lock:
            fresh = mask & ~self.faults
            self.faults |= mask
            if mask & STOPPING_FAULTS and self.armed:
                self.armed = False
                self._emit_event("disarmed", mask)
            if fresh:
                self._emit_event("fault", fresh)

    def simulate_interlock(self, ok: bool) -> None:
        with self._lock:
            if ok == self.interlock_ok:
                return
            self.interlock_ok = ok
            if not ok:
                self.faults |= 0x08
            self._emit_event("interlock", int(ok))

    def set_rail_mv(self, mv: int) -> None:
        self.rail_mv = mv

    # ---------------------------------------------------------------- internals
    def _trigger(self, t: int, software: bool) -> McuShotTrigger:
        s = self._sched
        self.seq += 1
        inhibited = not self.interlock_ok
        if software:
            self.trig_sw += 1
        else:
            self.trig_hw += 1
        if inhibited:
            self.trig_inhibited += 1
        self._last_trig = t
        if self.config.mode == "burst":
            starts = [t + d for d in s.exposure_start_rel_us]
            free = False
        else:
            p = self.config.frame_period_us
            k = (t - self._epoch) // p if t >= self._epoch else 0
            d0 = self._epoch + k * p - t
            starts = [t + d0, t + d0 + p]
            free = True
        offsets = [] if inhibited else list(s.pulse_offsets_us)
        if not inhibited and self.config.mode == "burst":
            self.pulses_total += s.n_exposures * s.n_pulses
        trig = McuShotTrigger(seq=self.seq, t_us=t, source="software" if software else "hardware",
                              exposure_start_us=starts, pulse_offsets_us=offsets,
                              strobe_inhibited=inhibited, free_run=free)
        self.triggers.append(trig)
        for cb in list(self._cbs):
            cb(trig)
        return trig
