"""Synchronous helpers: render one burst for a ground-truth shot and run the
pipeline on it (used by the tests, the accuracy report and the demo)."""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
from gs_common import ShotTrigger, StrobeConfig

from ..capture import plan_strobe, pulse_offsets
from ..pipeline import ProcessorConfig, ShotProcessor, ShotResult, detect_rest
from .rig import SyntheticScene
from .scene import ShotTruth, perturbed_rig

PREVIEW_PULSE_S = 10e-6


def render_burst(scene: SyntheticScene, truth: ShotTruth, strobe: StrobeConfig, trigger_delay_s: float = 0.0006,
                 rest: np.ndarray | None = None) -> tuple[dict[str, list[np.ndarray]], ShotTrigger]:
    """Ball struck at t = -trigger_delay_s, MCU trigger at t = 0 (MCU time in us)."""
    scene.place_ball(rest)
    scene.launch(truth, -trigger_delay_s)
    starts = [strobe.trigger_delay_us + k * strobe.frame_period_us for k in range(strobe.burst_exposures)]
    offs = pulse_offsets(strobe)
    trig = ShotTrigger(1, 0, "hardware", starts, offs)
    frames: dict[str, list[np.ndarray]] = {}
    for cam in scene.true_rig.pair:
        frames[cam.name] = [scene.render(cam.name, [(s + o + 0.5 * strobe.strobe_width_us) * 1e-6 for o in offs],
                                         strobe.strobe_width_us * 1e-6) for s in starts]
    return frames, trig


def render_rest(scene: SyntheticScene, rest: np.ndarray | None = None) -> dict[str, np.ndarray]:
    scene.place_ball(rest)
    return {c.name: scene.render(c.name, [0.0], PREVIEW_PULSE_S) for c in scene.true_rig.pair}


@dataclass
class Outcome:
    truth: ShotTruth
    result: ShotResult
    err: dict[str, float]
    rest_err_mm: float
    wall_s: float


def run_shot(scene: SyntheticScene, est_rig, truth: ShotTruth, speed_hint: float | None = None,
             seed: int = 0, trigger_delay_s: float = 0.0006, proc_cfg: ProcessorConfig | None = None,
             use_rest_detection: bool = True) -> Outcome:
    rng = np.random.default_rng(seed)
    rest_true = np.array([rng.normal(0, 0.003), rng.normal(0, 0.003), 0.0])
    t0 = time.perf_counter()
    rest = np.zeros(3)
    rest_err = float("nan")
    if use_rest_detection:
        X, _ = detect_rest(render_rest(scene, rest_true), est_rig)
        if X is not None:
            rest = X
            rest_err = float(np.linalg.norm(X - rest_true) * 1e3)
    plan = plan_strobe(speed_hint if speed_hint is not None else truth.speed_mps)
    frames, trig = render_burst(scene, truth, plan.config, trigger_delay_s, rest_true)
    proc = ShotProcessor(est_rig, proc_cfg)
    res = proc.process(frames, trig, plan.config, rest)
    wall = time.perf_counter() - t0
    err: dict[str, float] = {}
    if res.shot is not None:
        b = res.shot.ball
        err = {"speed": b.speed_mps - truth.speed_mps, "vla": b.vla_deg - truth.vla_deg,
               "hla": b.hla_deg - truth.hla_deg}
        if not truth.rolling and truth.total_spin_rpm > 0:
            err["spin"] = b.total_spin_rpm - truth.total_spin_rpm
            err["axis"] = b.spin_axis_deg - truth.spin_axis_deg
    return Outcome(truth, res, err, rest_err, wall)


def make_world(seed: int = 0, marking: str = "logo", calib_error: float = 1.0, sensor=None, ball_spec=None):
    from .ball import BallSpec
    scene = SyntheticScene(ball=ball_spec or BallSpec(marking=marking), sensor=sensor, seed=seed)
    est = perturbed_rig(scene.true_rig, np.random.default_rng(seed + 1000), calib_error) if calib_error > 0 \
        else scene.true_rig
    return scene, est
