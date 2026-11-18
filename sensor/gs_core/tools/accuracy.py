"""Accuracy and timing report on synthetic shots.

    python -m gs_core.tools.accuracy [--n 4] [--seed 0] [--ball logo|none|dots] [--dimple-depth-mm 0.2]

For each shot type, N shots with random ball orientation, rest offset,
sensor noise and an independently perturbed calibration are rendered and
processed; the table lists mean/RMS/max errors, the fraction with measured
spin and the processing time.
"""
from __future__ import annotations

import argparse
import time

import numpy as np

from ..synthetic.ball import BallSpec
from ..synthetic.harness import make_world, run_shot
from ..synthetic.render import SensorSpec
from ..synthetic.scene import ShotTruth

TYPES = {
    "driver": lambda r: ShotTruth(75 + r.normal(0, 2), 11 + r.normal(0, 1), r.uniform(-8, 8), 2700 + r.normal(0, 200), r.uniform(-20, 20)),
    "iron7": lambda r: ShotTruth(54 + r.normal(0, 2), 16 + r.normal(0, 1), r.uniform(-8, 8), 7000 + r.normal(0, 400), r.uniform(-20, 20)),
    "wedge": lambda r: ShotTruth(40 + r.normal(0, 2), 28 + r.normal(0, 1), r.uniform(-8, 8), 9500 + r.normal(0, 400), r.uniform(-20, 20)),
    "chip": lambda r: ShotTruth(15 + r.normal(0, 1), 30 + r.normal(0, 3), r.uniform(-5, 5), 3500 + r.normal(0, 300), r.uniform(-10, 10)),
    "putt": lambda r: ShotTruth(3 + r.normal(0, 0.5), 0.0, r.uniform(-4, 4), rolling=True, club=False),
}


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--ball", default="logo")
    ap.add_argument("--dimple-depth-mm", type=float, default=0.20)
    ap.add_argument("--types", default=",".join(TYPES))
    ap.add_argument("--psf-px", type=float, default=0.7, help="optical blur sigma (defocus)")
    ap.add_argument("--electrons", type=float, default=450.0, help="signal per pixel per pulse")
    a = ap.parse_args(argv)
    rows = []
    for ti, name in enumerate(a.types.split(",")):
        errs: dict[str, list[float]] = {}
        meas, ok, times = 0, 0, []
        for i in range(a.n):
            seed = a.seed * 1000 + ti * 100 + i
            rng = np.random.default_rng(seed)
            spec = BallSpec(marking=a.ball, dimple_depth_m=a.dimple_depth_mm * 1e-3,
                            seed=seed)
            sensor = SensorSpec(psf_sigma_px=a.psf_px, ball_electrons=a.electrons)
            scene, est = make_world(seed=seed, ball_spec=spec, sensor=sensor)
            truth = TYPES[name](rng)
            o = run_shot(scene, est, truth, seed=seed)
            if o.result.shot is None:
                continue
            ok += 1
            times.append(o.result.timings["total_s"])
            meas += int(o.result.shot.quality.spin_measured)
            for k, v in o.err.items():
                if k in ("spin", "axis") and not o.result.shot.quality.spin_measured:
                    continue
                errs.setdefault(k, []).append(v)
        rows.append((name, ok, meas, errs, times))
    print(f"{'type':8} {'ok':>4} {'spin meas':>9} | " + " | ".join(f"{k + ' (mean/rms/max)':>28}" for k in ("speed m/s", "vla deg", "hla deg", "spin rpm", "axis deg")) + " | time s")
    for name, ok, meas, errs, times in rows:
        cells = []
        for k in ("speed", "vla", "hla", "spin", "axis"):
            v = np.array(errs.get(k, []))
            cells.append("-".center(28) if v.size == 0 else
                         f"{v.mean():+8.2f} rms {np.sqrt(np.mean(v ** 2)):6.2f} max {np.abs(v).max():6.2f}".rjust(28))
        t = f"{np.mean(times):.2f}/{np.max(times):.2f}" if times else "-"
        print(f"{name:8} {ok:>2}/{a.n} {meas:>5}/{ok:<3} | " + " | ".join(cells) + f" | {t}")


if __name__ == "__main__":
    t0 = time.time()
    main()
    print(f"wall {time.time() - t0:.0f} s")
