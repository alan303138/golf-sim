"""gs-core-demo: render one synthetic shot, process it, save the frames.

    gs-core-demo [--shot driver|iron7|wedge|chip|putt] [--hint SPEED] [--out DIR]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from .capture import plan_strobe
from .pipeline import ShotProcessor
from .synthetic.harness import make_world, render_burst
from .synthetic.scene import PRESETS


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shot", default="driver", choices=sorted(PRESETS))
    ap.add_argument("--hint", type=float, default=None, help="strobe speed hint (default: true speed)")
    ap.add_argument("--out", default="gs_demo")
    a = ap.parse_args(argv)
    scene, est = make_world(seed=0)
    truth = PRESETS[a.shot]
    plan = plan_strobe(a.hint if a.hint else truth.speed_mps)
    frames, trig = render_burst(scene, truth, plan.config)
    res = ShotProcessor(est).process(frames, trig, plan.config, np.zeros(3))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for name, fl in frames.items():
        for k, f in enumerate(fl):
            img = np.clip((f.astype(np.float32) - 60) / 500 * 255, 0, 255).astype(np.uint8)
            for b in res.images.get(name, [[]] * len(fl))[k] if k < len(res.images.get(name, [])) else []:
                cv2.circle(img, (int(round(b.u)), int(round(b.v))), int(round(b.r_px)), 255, 1)
            cv2.imwrite(str(out / f"{name}_exp{k}.png"), img)
    print("truth :", truth)
    print("shot  :", json.dumps(res.shot.to_dict()["ball"] if res.shot else None, indent=1))
    print("spin  :", None if res.spin is None else {k: (round(float(v), 3) if isinstance(v, (float, np.floating)) else v)
                                                  for k, v in res.spin.details.items()})
    print("timing:", {k: round(v, 3) for k, v in res.timings.items()}, "| frames in", out)


if __name__ == "__main__":
    main()
