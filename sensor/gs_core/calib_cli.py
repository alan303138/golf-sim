"""gs-calib: calibration command-line tool.

    gs-calib intrinsics --board charuco:8x6:0.030:0.022 --out cam0_intr.json cam0_*.png
    gs-calib stereo     --board ... --intr0 cam0_intr.json --intr1 cam1_intr.json \
                        --pairs 'pairs/cam0_*.png' 'pairs/cam1_*.png' --out stereo.json
    gs-calib align      --board ... --intr0 ... --intr1 ... --stereo stereo.json \
                        --mat cam0_mat.png --ball-xy 0.12 0.09 [--gravity gx gy gz] \
                        [--lights lights_cam0.json] --out /var/lib/golfsim/calibration.json
    gs-calib check      --calib calibration.json --board ... cam0_check.png cam1_check.png
    gs-calib show       --calib calibration.json

Images are 8- or 16-bit PNG files captured with the ball cameras (lit by the
IR strobe; see README for the capture procedure).
"""
from __future__ import annotations

import argparse
import glob
import json
import sys

import cv2
import numpy as np

from .calibration import (RigCalibration, StereoResult, align_world_from_board,
                          calibrate_intrinsics, calibrate_stereo, check_with_board, parse_board)
from .geometry import CameraModel


def _read(path: str) -> np.ndarray:
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise SystemExit(f"cannot read {path}")
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def _load_intr(path: str, name: str) -> CameraModel:
    d = json.loads(open(path).read())
    return CameraModel(name, np.array(d["K"]), np.array(d["dist"]), tuple(d["image_size"]))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="gs-calib", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("intrinsics")
    p.add_argument("--board", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("images", nargs="+")
    p = sub.add_parser("stereo")
    p.add_argument("--board", required=True)
    p.add_argument("--intr0", required=True)
    p.add_argument("--intr1", required=True)
    p.add_argument("--pairs", nargs=2, required=True, metavar=("GLOB0", "GLOB1"))
    p.add_argument("--out", required=True)
    p = sub.add_parser("align")
    for a in ("--board", "--intr0", "--intr1", "--stereo", "--mat", "--out"):
        p.add_argument(a, required=True)
    p.add_argument("--ball-xy", nargs=2, type=float, required=True, help="board coordinates (m) of the ball rest point")
    p.add_argument("--board-thickness", type=float, default=0.0)
    p.add_argument("--gravity", nargs=3, type=float, help="gravity in the cam0 frame (from the tilt sensor)")
    p.add_argument("--lights", help="JSON list of LED positions in the cam0 frame (m)")
    p = sub.add_parser("check")
    p.add_argument("--calib", required=True)
    p.add_argument("--board", required=True)
    p.add_argument("image0")
    p.add_argument("image1")
    p.add_argument("--at-reference", action="store_true", help="board lies at its alignment position")
    p.add_argument("--ball-xy", nargs=2, type=float, default=(0.0, 0.0))
    p = sub.add_parser("show")
    p.add_argument("--calib", required=True)
    a = ap.parse_args(argv)

    if a.cmd == "intrinsics":
        board = parse_board(a.board)
        files = sorted(f for g in a.images for f in glob.glob(g))
        r = calibrate_intrinsics([_read(f) for f in files], board)
        out = {"K": r.K.tolist(), "dist": r.dist.tolist(), "image_size": list(r.image_size), "rms_px": r.rms_px,
               "n_views": r.n_views, "per_view_rms_px": r.per_view_rms}
        open(a.out, "w").write(json.dumps(out, indent=2))
        print(f"rms {r.rms_px:.3f} px over {r.n_views} views (want < 0.3 px); worst view "
              f"{max(r.per_view_rms):.3f} px")
    elif a.cmd == "stereo":
        board = parse_board(a.board)
        f0, f1 = sorted(glob.glob(a.pairs[0])), sorted(glob.glob(a.pairs[1]))
        if len(f0) != len(f1):
            raise SystemExit("pair globs must match the same number of files")
        i0, i1 = _load_intr(a.intr0, "cam0"), _load_intr(a.intr1, "cam1")
        r = calibrate_stereo([(_read(x), _read(y)) for x, y in zip(f0, f1)], board, i0, i1)
        open(a.out, "w").write(json.dumps({"R": r.R.tolist(), "T": r.T.tolist(), "rms_px": r.rms_px,
                                           "n_views": r.n_views}, indent=2))
        print(f"stereo rms {r.rms_px:.3f} px, baseline {r.baseline_m * 1000:.2f} mm, {r.n_views} pairs")
    elif a.cmd == "align":
        board = parse_board(a.board)
        i0, i1 = _load_intr(a.intr0, "cam0"), _load_intr(a.intr1, "cam1")
        sd = json.loads(open(a.stereo).read())
        st = StereoResult(np.array(sd["R"]), np.array(sd["T"]), sd["rms_px"], sd["n_views"])
        lights = np.array(json.loads(open(a.lights).read()), float) if a.lights else None
        rig = align_world_from_board(i0, i1, st, _read(a.mat), board, a.ball_xy, a.board_thickness,
                                     np.array(a.gravity) if a.gravity else None, lights)
        rig.save(a.out)
        print(json.dumps(rig.summary(), indent=2))
    elif a.cmd == "check":
        rig = RigCalibration.load(a.calib)
        rep = check_with_board(rig, _read(a.image0), _read(a.image1), parse_board(a.board),
                               board_at_reference=a.at_reference, ball_xy_on_board=a.ball_xy)
        print(json.dumps({"ok": rep.ok, "moved": rep.moved, **rep.details}, indent=2))
        return 0 if rep.ok else 1
    elif a.cmd == "show":
        print(json.dumps(RigCalibration.load(a.calib).summary(), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
