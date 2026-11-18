"""Strobe timing and capture policies.

Which capture strategy works best cannot be decided before the real cameras
and trigger sensors exist; the three policies below are interchangeable and
expose the same interface (``strobe_config``), and the analysis helpers make
the trade-offs explicit (see README).

All policies share the strobe planner: the strobe interval is chosen so that
consecutive ball images are ~1.3 ball diameters apart (just separated, as many
images as possible in the ~250 mm field), from a speed hint (club selection or
previous shot); without a hint a safe default is used. Slow shots (putts)
use one pulse per exposure and successive frames instead.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
from gs_common import StrobeConfig

from .geometry import BALL_DIAMETER_M

FIELD_LENGTH_M = 0.25          # flight length with the ball fully in both views (<= 30 deg launch)
TARGET_SPACING_D = 1.3
MAX_DUTY = 0.02                # firmware/hardware limit, strobe on-time / time
SINGLE_PULSE_BELOW_MPS = 6.0
DEFAULT_SPEED_HINT = 55.0      # without a hint: spacing 1.3 D at 55 m/s, ~1.75 D at 75 m/s, ~1 D at 40 m/s
READOUT_US = 16_600            # IMX296 full frame at ~60 fps - VERIFY with the real module


# typical ball speed per club (m/s), used as a hint before the first shot with that club
CLUB_SPEED_HINT = {"driver": 70.0, "wood": 62.0, "hybrid": 58.0, "iron": 52.0, "iron7": 52.0,
                   "wedge": 40.0, "chip": 15.0, "putter": 3.0, "putt": 3.0}


@dataclass
class StrobePlan:
    config: StrobeConfig
    speed_hint: float
    spacing_d: float              # expected image spacing at the hint speed, in ball diameters
    single_pulse: bool


def plan_strobe(speed_hint: float | None, base: StrobeConfig | None = None, strobe_width_us: int = 10,
                acoustic_delay_us: int = 600, field_m: float = FIELD_LENGTH_M) -> StrobePlan:
    """Strobe/exposure timing for a burst triggered at (about) impact.

    ``acoustic_delay_us``: time from impact to the trigger reaching the MCU (an
    acoustic trigger 20 cm away needs ~0.6 ms; the ball has then flown ~5 cm
    at 80 m/s, still inside the field). It shortens the time left in the field
    for the first exposure.
    """
    b = replace(base or StrobeConfig())
    v = float(speed_hint) if speed_hint and speed_hint > 0 else DEFAULT_SPEED_HINT
    b.strobe_width_us = int(strobe_width_us)
    if v < SINGLE_PULSE_BELOW_MPS:
        # putt: one pulse per frame, successive frames; ~1.3 D per frame is impossible at
        # the sensor frame rate, so take frames as fast as readout allows
        b.pulses_per_exposure = 1
        b.first_pulse_delay_us = 50
        b.exposure_tail_us = 50
        b.strobe_interval_us = 1000
        b.frame_period_us = READOUT_US + 400
        travel = v * b.frame_period_us * 1e-6
        b.burst_exposures = int(np.clip(math.ceil(field_m / max(travel, 1e-3)), 3, 8))
        return StrobePlan(b, v, travel / BALL_DIAMETER_M, True)
    interval = TARGET_SPACING_D * BALL_DIAMETER_M / v
    b.strobe_interval_us = int(round(interval * 1e6))
    t_field = field_m / v - acoustic_delay_us * 1e-6
    n = int(np.clip(math.floor(t_field / interval) + 1, 3, 8))
    b.pulses_per_exposure = n
    b.first_pulse_delay_us = 30
    b.exposure_tail_us = 30
    # a second exposure catches what the first missed (late trigger, slow shot)
    b.burst_exposures = 2
    b.frame_period_us = max(READOUT_US + 400, exposure_us(b) + READOUT_US)
    duty = b.strobe_width_us * n / max(1, b.frame_period_us)
    if duty > MAX_DUTY:
        b.frame_period_us = int(math.ceil(b.strobe_width_us * n / MAX_DUTY))
    return StrobePlan(b, v, TARGET_SPACING_D, False)


def exposure_us(c: StrobeConfig) -> int:
    return c.first_pulse_delay_us + (c.pulses_per_exposure - 1) * c.strobe_interval_us + c.strobe_width_us \
        + c.exposure_tail_us


def pulse_offsets(c: StrobeConfig) -> list[int]:
    return [c.first_pulse_delay_us + k * c.strobe_interval_us for k in range(c.pulses_per_exposure)]


# ------------------------------------------------------------------ policies
class CapturePolicy:
    """Base: produces the strobe configuration for the MCU and says how a shot is triggered."""

    name = "base"
    mcu_mode = "burst"

    def strobe_config(self, speed_hint: float | None) -> StrobeConfig:
        return plan_strobe(speed_hint).config


class BurstPolicy(CapturePolicy):
    """(a) An external fast shot trigger (Pico TRIG_IN from the acoustic or optical
    front end, or ``McuLink.fire()``) starts a burst of exposures."""

    name = "burst"

    def __init__(self, acoustic_delay_us: int = 600) -> None:
        self.acoustic_delay_us = acoustic_delay_us

    def strobe_config(self, speed_hint: float | None) -> StrobeConfig:
        c = plan_strobe(speed_hint, acoustic_delay_us=self.acoustic_delay_us).config
        c.mode = "burst"
        c.trigger_delay_us = 0
        return c


class FreeRunPolicy(CapturePolicy):
    """(b) Continuous exposures; the host looks for a frame with ball images
    away from the rest position. See :func:`free_run_miss_probability`."""

    name = "free_run"
    mcu_mode = "free_run"

    def strobe_config(self, speed_hint: float | None) -> StrobeConfig:
        c = plan_strobe(speed_hint).config
        c.mode = "free_run"
        return c


class VisionTriggerPolicy(CapturePolicy):
    """(c) Watch a small crop around the resting ball at a high frame rate and
    request a burst (``McuLink.fire()``) when the ball leaves.

    Wired in ``PipelineShotSource(trigger_camera=...)``: a separate camera
    (or a separately configured stream) delivering crops; the first crop after
    READY is the reference, the burst is fired from the capture thread.
    Experimental: the trigger camera must run free with a short exposure while
    the burst needs it in trigger mode, and the departure must be seen, decided
    and signalled before the ball has left the field (see
    :func:`vision_trigger_budget`). Unlikely to work for full shots on a Pi 5
    unless the crop rate is several kHz.
    """

    name = "vision"

    def __init__(self, crop_fps: float = 1000.0) -> None:
        self.crop_fps = crop_fps

    def strobe_config(self, speed_hint: float | None) -> StrobeConfig:
        c = plan_strobe(speed_hint, acoustic_delay_us=int(vision_trigger_budget(self.crop_fps, 60.0)["latency_us"])).config
        c.mode = "burst"
        return c

    @staticmethod
    def departed(crop_now: np.ndarray, crop_rest: np.ndarray, frac: float = 0.15) -> bool:
        """Ball left its rest position: the bright-ball pixel count dropped by ``frac``."""
        thr = 0.5 * float(np.percentile(crop_rest, 99))
        a = float(np.count_nonzero(crop_rest > thr))
        b = float(np.count_nonzero(crop_now > thr))
        return a > 0 and b < (1 - frac) * a


POLICIES = {"burst": BurstPolicy, "free_run": FreeRunPolicy, "vision": VisionTriggerPolicy}


# ------------------------------------------------------------------ analysis
def free_run_dead_time(c: StrobeConfig, readout_us: int = READOUT_US, overlap: bool = False) -> float:
    """Fraction of time no exposure is open. Without exposure-during-readout
    (IMX296 in trigger mode is assumed NOT to overlap - verify), the next
    exposure can only start after the previous frame is read out."""
    exp = exposure_us(c)
    period = max(c.frame_period_us, exp if overlap else exp + readout_us)
    return 1.0 - exp / period


def free_run_miss_probability(speed: float, c: StrobeConfig, readout_us: int = READOUT_US,
                              min_images: int = 3, field_m: float = FIELD_LENGTH_M, overlap: bool = False) -> float:
    """Probability that fewer than ``min_images`` strobe images of a ball crossing
    the field land in the exposures (uniform random shot phase).

    Only pulses fired while the ball is inside the field count; pulses fire
    only inside exposures."""
    exp = exposure_us(c)
    period = max(c.frame_period_us, exp if overlap else exp + readout_us)
    t_field = field_m / speed * 1e6
    offs = np.array(pulse_offsets(c), float)
    pulses = (np.arange(-1, int(t_field // period) + 3)[:, None] * period + offs[None, :]).ravel()
    phases = np.linspace(0, period, 2000, endpoint=False)
    cnt = np.array([np.count_nonzero((pulses >= ph) & (pulses < ph + t_field)) for ph in phases])
    return float(np.mean(cnt < min_images))


def vision_trigger_budget(crop_fps: float, speed: float, decide_us: float = 300.0, usb_us: float = 150.0,
                          field_m: float = FIELD_LENGTH_M) -> dict[str, float]:
    """Latency of the vision trigger and the flight left for the burst.

    Detection waits on average 1.5 crop frames (exposure + readout of the frame
    that first shows the departure), then the host decides (frame delivery
    through libcamera + processing) and tells the Pico over USB."""
    frame_us = 1e6 / crop_fps
    latency = 1.5 * frame_us + decide_us + usb_us
    travel = speed * latency * 1e-6
    return {"latency_us": latency, "ball_travel_m": travel, "field_left_m": field_m - travel,
            "ok": float(travel < 0.5 * field_m)}
