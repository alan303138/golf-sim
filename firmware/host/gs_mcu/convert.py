"""Conversions between wire messages and the gs_common types."""
from __future__ import annotations

from dataclasses import dataclass

from gs_common.interfaces import McuInfo, ShotTrigger

from . import protocol as P
from .errors import fault_names


@dataclass
class McuShotTrigger(ShotTrigger):
    """ShotTrigger plus the MCU-specific flags. Still an instance of ShotTrigger."""
    strobe_inhibited: bool = False   # STROBE_EN was low: no strobe pulses were generated
    free_run: bool = False           # free_run event: exposure_start_us holds the exposure running
                                     # at (or just before) the event and the next one
    pulse_width_us: int = 0


def trigger_from_msg(m: P.TriggerMsg) -> McuShotTrigger:
    return McuShotTrigger(
        seq=m.seq,
        t_us=m.t_us,
        source=P.SRC_NAMES.get(m.source, f"source{m.source}"),
        exposure_start_us=[m.t_us + d for d in m.exp_delta_us],
        pulse_offsets_us=list(m.pulse_offset_us),
        strobe_inhibited=bool(m.flags & P.TRIG_FLAG_INHIBITED),
        free_run=bool(m.flags & P.TRIG_FLAG_FREE_RUN),
    )


def info_from_msg(m: P.InfoMsg) -> McuInfo:
    return McuInfo(fw_version=m.fw_version, hw_revision=m.hw_revision, serial=m.serial, uptime_ms=m.uptime_ms)


def status_to_dict(s: P.StatusMsg, fw_version: str = "") -> dict:
    return {
        "armed": s.armed,
        "mode": P.MODE_NAMES.get(s.mode, str(s.mode)),
        "faults": fault_names(s.faults),
        "faults_mask": s.faults,
        "interlock_ok": s.interlock_ok,
        "strobe_forced_low": bool(s.flags & P.STATUS_FLAG_STROBE_FORCED_LOW),
        "rail_mv": s.rail_mv,
        "rail_v": s.rail_mv / 1000.0,
        "trigger_hw": s.trig_hw,
        "trigger_sw": s.trig_sw,
        "trigger_inhibited": s.trig_inhibited,
        "bursts_done": s.bursts_done,
        "pulses_total": s.pulses_total,
        "max_pulse_us_seen": s.max_pulse_us_seen,
        "duty_ppm_now": s.duty_ppm_now,
        "rx_bad_frames": s.rx_bad_frames,
        "uptime_ms": s.uptime_ms,
        "fw_version": fw_version,
    }
