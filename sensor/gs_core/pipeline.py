"""Shot processing: frames of one burst + MCU timing -> ball launch data.

``ShotProcessor.process`` is synchronous and CPU-bound; the shot sources run
it in a worker thread.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from gs_common import BallData, Quality, Shot, ShotTrigger, StrobeConfig

from .ballfit import BallImage, FitConfig, estimate_background, find_ball_images, prepare_foreground
from .calibration import RigCalibration, midpoint_triangulate
from .camera import Frame
from .spin import BallChart, SpinConfig, SpinEstimate, estimate_spin, make_chart
from .spin_model import empirical_spin_rpm, plausible_spin_limit_rpm
from .track import LaunchEstimate, Slot, TrackConfig, _visible, associate, stereo_match

MIN_SPIN_SPEED = 6.0         # below: putt/roll, spin not measured


@dataclass
class ProcessorConfig:
    fit: FitConfig = field(default_factory=FitConfig)
    track: TrackConfig = field(default_factory=TrackConfig)
    spin: SpinConfig = field(default_factory=SpinConfig)
    max_spin_charts_per_cam: int = 4


@dataclass
class ShotResult:
    shot: Shot | None
    launch: LaunchEstimate | None
    spin: SpinEstimate | None
    timings: dict[str, float]
    images: dict[str, list[list[BallImage]]]
    reason: str = ""


def slots_from_trigger(trig: ShotTrigger, strobe: StrobeConfig) -> list[Slot]:
    """Strobe pulse centre times (s, MCU clock relative to the trigger event)."""
    out = []
    for e, t0 in enumerate(trig.exposure_start_us):
        for k, off in enumerate(trig.pulse_offsets_us):
            out.append(Slot(e, k, (t0 + off - trig.t_us + 0.5 * strobe.strobe_width_us) * 1e-6))
    return out


def detect_rest(frames: dict[str, np.ndarray], rig: RigCalibration, black: float = 0.0,
                cfg: FitConfig | None = None, search_m: float = 0.06) -> tuple[np.ndarray | None, dict[str, Any]]:
    """Ball at rest from one frame per camera (lit by one strobe pulse).

    Returns (centre in world, info). The ball must be within ``search_m`` of
    the calibrated origin and touch the mat (|z| < 6 mm)."""
    imgs = {}
    for cam in rig.pair:
        fg = prepare_foreground(frames[cam.name], estimate_background(frames[cam.name]))
        found = find_ball_images(fg, cam, cam.to_cam(rig.lights), cfg)
        imgs[cam.name] = found
    best = None
    for a in imgs[rig.cam0.name]:
        for b in imgs[rig.cam1.name]:
            X, gap = midpoint_triangulate(rig.cam0.center, a.ray_w, rig.cam1.center, b.ray_w)
            if gap < 0.002 and np.linalg.norm(X[:2]) < search_m and abs(X[2]) < 0.006:
                score = gap + 0.1 * float(np.linalg.norm(X))
                if best is None or score < best[0]:
                    best = (score, X, gap, a, b)
    if best is None:
        return None, {"n0": len(imgs[rig.cam0.name]), "n1": len(imgs[rig.cam1.name])}
    _, X, gap, a, b = best
    return X, {"gap_mm": float(gap) * 1e3, "rms_px": [a.rms_px, b.rms_px], "position_mm": (X * 1e3).round(2).tolist()}


class ShotProcessor:
    def __init__(self, rig: RigCalibration, config: ProcessorConfig | None = None) -> None:
        self.rig = rig
        self.cfg = config or ProcessorConfig()

    # ----------------------------------------------------------------------
    def process(self, frames: dict[str, list[Frame | np.ndarray]], trig: ShotTrigger, strobe: StrobeConfig,
                rest: np.ndarray | None = None, black_level: float | dict[str, float] = 0.0,
                full_scale: int = 1023) -> ShotResult:
        t0 = time.perf_counter()
        tim: dict[str, float] = {}
        rig = self.rig
        names = [c.name for c in rig.pair]
        n_exp = min(len(frames[n]) for n in names)
        slots = slots_from_trigger(trig, strobe)
        imgs: dict[str, list[list[BallImage]]] = {n: [] for n in names}
        fgs: dict[str, list[np.ndarray]] = {n: [] for n in names}
        balls = []
        for e in range(n_exp):
            per = []
            for cam in rig.pair:
                fr = frames[cam.name][e]
                raw = fr.image if isinstance(fr, Frame) else fr
                bg = estimate_background(raw)
                fg = prepare_foreground(raw, bg)
                # saturated pixels carry no shading information: mark them for the spin charts
                fg_sp = fg.copy()
                fg_sp[raw >= full_scale - 2] = -2e6
                fgs[cam.name].append(fg_sp)
                found = find_ball_images(fg, cam, cam.to_cam(rig.lights), self.cfg.fit)
                imgs[cam.name].append(found)
                per.append(found)
            balls += stereo_match(per[0], per[1], rig, e, self.cfg.track)
            if e < n_exp - 1 and self._done_early(balls, slots, rest, e):
                tim["exposures_skipped"] = n_exp - 1 - e
                break
        tim["detect_s"] = time.perf_counter() - t0
        t1 = time.perf_counter()
        launch = associate(balls, slots, rig, rest, self.cfg.track)
        tim["track_s"] = time.perf_counter() - t1
        if launch is None:
            tim["total_s"] = time.perf_counter() - t0
            return ShotResult(None, None, None, tim, imgs, reason=f"no trajectory ({len(balls)} stereo balls)")
        t2 = time.perf_counter()
        spin = None
        if launch.speed_mps >= MIN_SPIN_SPEED and launch.trajectory.model == "ballistic":
            spin = self._spin(launch, slots, imgs, fgs)
        tim["spin_s"] = time.perf_counter() - t2
        shot = self._make_shot(launch, spin)
        tim["total_s"] = time.perf_counter() - t0
        return ShotResult(shot, launch, spin, tim, imgs)

    def _done_early(self, balls, slots: list[Slot], rest, e: int) -> bool:
        """Skip later exposures when the ball is already well measured and the fit
        says it is out of view by then (saves ~0.15 s per frame pair)."""
        L = associate(list(balls), slots, self.rig, rest, self.cfg.track)
        if L is None or L.trajectory.n < 4:
            return False
        later = np.array([s.t for s in slots if s.exposure > e])
        if later.size == 0:
            return True
        P = L.trajectory.position(later)
        return not bool(np.any(_visible(self.rig, P, 0.0)))

    # ----------------------------------------------------------------------
    def _spin(self, launch: LaunchEstimate, slots: list[Slot], imgs: dict[str, list[list[BallImage]]],
              fgs: dict[str, list[np.ndarray]]) -> SpinEstimate | None:
        rig = self.rig
        tr = launch.trajectory
        charts: list[BallChart] = []
        for k, cam in enumerate(rig.pair):
            used = []
            for sb in launch.balls:
                im = sb.a if k == 0 else sb.b
                if im.clipped or im.arc_frac < 0.5:
                    continue
                used.append((sb, im))
            # prefer well-measured, unclipped images; keep the time order
            used = sorted(used, key=lambda x: -x[1].arc_frac)[: self.cfg.max_spin_charts_per_cam]
            used.sort(key=lambda x: slots[x[0].slot].t)
            for sb, im in used:
                t = slots[sb.slot].t
                others = [(o.u, o.v, o.r_px) for o in imgs[cam.name][sb.exposure] if o is not im
                          and math.hypot(o.u - im.u, o.v - im.v) < 2.05 * im.r_px]
                ch = make_chart(fgs[cam.name][sb.exposure], cam, tr.position(t), t, rig.lights, 1e9,
                                exclude=others, cfg=self.cfg.spin)
                charts.append(ch)
        if len(charts) < 2:
            return None
        v_dir = launch.velocity / np.linalg.norm(launch.velocity)
        return estimate_spin(charts, v_dir, plausible_spin_limit_rpm(launch.speed_mps, launch.vla_deg), self.cfg.spin)

    def _make_shot(self, L: LaunchEstimate, spin: SpinEstimate | None) -> Shot:
        n = L.trajectory.n
        launch_conf = float(np.clip((n - 1) / 3.0, 0, 1) * math.exp(-L.trajectory.rms_m / 0.0015)
                            * math.exp(-L.sigma_speed / max(0.5, 0.03 * L.speed_mps)))
        rolling = L.trajectory.model == "rolling"
        if spin is not None and spin.measured:
            total, axis, measured = spin.total_rpm, spin.axis_deg, True
            conf = launch_conf * (0.7 + 0.3 * spin.confidence)
        else:
            total = 0.0 if rolling or L.speed_mps < MIN_SPIN_SPEED else empirical_spin_rpm(L.speed_mps, L.vla_deg)
            axis, measured = 0.0, False
            conf = launch_conf * 0.7
        ball = BallData(speed_mps=round(L.speed_mps, 3), vla_deg=round(L.vla_deg, 3), hla_deg=round(L.hla_deg, 3),
                        total_spin_rpm=round(total, 1), spin_axis_deg=round(axis, 2))
        return Shot(ball=ball, club=None, quality=Quality(confidence=round(conf, 3), sources=["stereo"],
                                                          spin_measured=measured))
