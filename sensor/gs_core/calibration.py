"""Calibration: intrinsics, stereo extrinsics, world alignment, save/load, check.

Operator procedure (details in README):

1. Intrinsics per camera from ~20 views of a ChArUco board or checkerboard
   (``calibrate_intrinsics``).
2. Stereo extrinsics (cam1 relative to cam0) from synchronised views of the
   same board (``calibrate_stereo``), intrinsics fixed.
3. World alignment (``align_world_from_board``): the board lies flat on the
   mat, a known board point at the ball rest position and the board +x axis
   along the target line. The contract frame is: origin = ball centre at rest
   (one ball radius above the mat), +X target line, +Y left, +Z up. When the
   unit's tilt sensor is available, its gravity vector defines +Z instead of
   the board normal (a mat that is not level then does not tilt the frame).
4. ``check_with_board`` / ``check_with_ball`` report reprojection error and
   whether the rig moved since calibration.

The calibration file is JSON (human-readable, diffable).
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np

from .geometry import BALL_RADIUS_M, CameraModel

CALIB_VERSION = 2
CAM_NAMES = ("cam0", "cam1")


# ------------------------------------------------------------------- boards
@dataclass
class CheckerboardSpec:
    cols: int = 9            # inner corners per row
    rows: int = 6            # inner corners per column
    square_m: float = 0.025

    def object_points(self) -> np.ndarray:
        g = np.zeros((self.rows * self.cols, 3), np.float32)
        g[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) * self.square_m
        return g


@dataclass
class CharucoSpec:
    squares_x: int = 11
    squares_y: int = 8
    square_m: float = 0.030
    marker_m: float = 0.022
    dictionary: str = "DICT_5X5_100"

    def board(self) -> Any:
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary))
        return cv2.aruco.CharucoBoard((self.squares_x, self.squares_y), self.square_m, self.marker_m, d)


BoardSpec = CheckerboardSpec | CharucoSpec


def parse_board(spec: str) -> BoardSpec:
    """'checker:9x6:0.025' or 'charuco:11x8:0.030:0.022[:DICT_5X5_100]'."""
    parts = spec.split(":")
    kind = parts[0].lower()
    nx, ny = (int(v) for v in parts[1].lower().split("x"))
    if kind.startswith("check"):
        return CheckerboardSpec(nx, ny, float(parts[2]))
    if kind == "charuco":
        d = parts[4] if len(parts) > 4 else "DICT_5X5_100"
        return CharucoSpec(nx, ny, float(parts[2]), float(parts[3]), d)
    raise ValueError(f"unknown board spec {spec!r}")


def detect_board(image: np.ndarray, board: BoardSpec) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """Find board corners. Returns (object points (N,3), image points (N,2), ids (N,)) or None."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if gray.dtype != np.uint8:
        gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    if isinstance(board, CheckerboardSpec):
        ok, corners = cv2.findChessboardCornersSB(gray, (board.cols, board.rows),
                                                  cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            ok, corners = cv2.findChessboardCorners(gray, (board.cols, board.rows),
                                                    cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not ok:
                return None
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4))
        img = corners.reshape(-1, 2).astype(np.float64)
        obj = board.object_points().astype(np.float64)
        return obj, img, np.arange(len(obj))
    b = board.board()
    det = cv2.aruco.CharucoDetector(b)
    ch_corners, ch_ids, _, _ = det.detectBoard(gray)
    if ch_ids is None or len(ch_ids) < 6:
        return None
    ids = ch_ids.ravel()
    obj = np.asarray(b.getChessboardCorners())[ids].astype(np.float64)
    return obj, ch_corners.reshape(-1, 2).astype(np.float64), ids


@dataclass
class IntrinsicsResult:
    K: np.ndarray
    dist: np.ndarray
    image_size: tuple[int, int]
    rms_px: float
    n_views: int
    per_view_rms: list[float] = field(default_factory=list)


def calibrate_intrinsics(images: Sequence[np.ndarray], board: BoardSpec,
                         fix_k3: bool = True) -> IntrinsicsResult:
    obj_l, img_l = [], []
    size = None
    for im in images:
        size = (im.shape[1], im.shape[0])
        r = detect_board(im, board)
        if r is None:
            continue
        obj_l.append(r[0].astype(np.float32))
        img_l.append(r[1].astype(np.float32).reshape(-1, 1, 2))
    if len(obj_l) < 3 or size is None:
        raise ValueError(f"board found in only {len(obj_l)} images; need >= 3 (recommended 15-25)")
    flags = cv2.CALIB_FIX_K3 if fix_k3 else 0
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_l, img_l, size, None, None, flags=flags)
    per = []
    for o, i, rv, tv in zip(obj_l, img_l, rvecs, tvecs):
        p, _ = cv2.projectPoints(o, rv, tv, K, dist)
        per.append(float(np.sqrt(np.mean(np.sum((p - i) ** 2, axis=2)))))
    return IntrinsicsResult(K, dist.ravel()[:5], size, float(rms), len(obj_l), per)


@dataclass
class StereoResult:
    R: np.ndarray     # camera A -> camera B rotation (X_b = R X_a + T)
    T: np.ndarray
    rms_px: float
    n_views: int

    @property
    def baseline_m(self) -> float:
        return float(np.linalg.norm(self.T))


def calibrate_stereo(pairs: Sequence[tuple[np.ndarray, np.ndarray]], board: BoardSpec,
                     intr_a: IntrinsicsResult | CameraModel, intr_b: IntrinsicsResult | CameraModel) -> StereoResult:
    obj_l, ia_l, ib_l = [], [], []
    size = None
    for ia, ib in pairs:
        size = (ia.shape[1], ia.shape[0])
        ra, rb = detect_board(ia, board), detect_board(ib, board)
        if ra is None or rb is None:
            continue
        common, ka, kb = np.intersect1d(ra[2], rb[2], return_indices=True)
        if len(common) < 6:
            continue
        obj_l.append(ra[0][ka].astype(np.float32))
        ia_l.append(ra[1][ka].astype(np.float32).reshape(-1, 1, 2))
        ib_l.append(rb[1][kb].astype(np.float32).reshape(-1, 1, 2))
    if len(obj_l) < 3:
        raise ValueError(f"board seen by both cameras in only {len(obj_l)} pairs; need >= 3")
    rms, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
        obj_l, ia_l, ib_l, intr_a.K, intr_a.dist, intr_b.K, intr_b.dist, size,
        flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9))
    return StereoResult(R, T.ravel(), float(rms), len(obj_l))


# --------------------------------------------------------- rig calibration
@dataclass
class RigCalibration:
    """Calibrated stereo pair and strobe geometry in the contract's world frame.

    ``lights``: (N,3) world positions of the IR LED boards (from the mechanical
    design, transformed with the cameras by the world alignment). They are
    used by the shading-aware limb fit and by the spin estimator.
    """

    cameras: dict[str, CameraModel]
    lights: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    meta: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.lights = np.asarray(self.lights, float).reshape(-1, 3)

    @property
    def cam0(self) -> CameraModel:
        return self.cameras[CAM_NAMES[0]]

    @property
    def cam1(self) -> CameraModel:
        return self.cameras[CAM_NAMES[1]]

    @property
    def pair(self) -> tuple[CameraModel, CameraModel]:
        return self.cam0, self.cam1

    @property
    def baseline_m(self) -> float:
        return float(np.linalg.norm(self.cam0.center - self.cam1.center))

    def to_dict(self) -> dict[str, Any]:
        return {"version": CALIB_VERSION,
                "cameras": {k: v.to_dict() for k, v in self.cameras.items()},
                "lights": self.lights.tolist(), "meta": self.meta}

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "RigCalibration":
        if d.get("version") != CALIB_VERSION:
            raise ValueError(f"unsupported calibration version {d.get('version')}")
        return cls({k: CameraModel.from_dict(v) for k, v in d["cameras"].items()},
                   np.array(d.get("lights", []), float).reshape(-1, 3), d.get("meta", {}))

    def save(self, path: str | Path) -> None:
        p = Path(path)
        tmp = p.with_suffix(p.suffix + ".tmp")
        tmp.write_text(json.dumps(self.to_dict(), indent=2))
        tmp.replace(p)

    @classmethod
    def load(cls, path: str | Path) -> "RigCalibration":
        return cls.from_dict(json.loads(Path(path).read_text()))

    def summary(self) -> dict[str, Any]:
        out: dict[str, Any] = {"baseline_m": round(self.baseline_m, 5)}
        for n, m in self.cameras.items():
            out[n] = {"position_m": np.round(m.center, 4).tolist(), "fx_px": round(m.fx, 2),
                      "optical_axis": np.round(m.R[2], 4).tolist()}
        out["meta"] = self.meta
        return out


# ---------------------------------------------------------------- geometry
def midpoint_triangulate(oa: np.ndarray, da: np.ndarray, ob: np.ndarray, db: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Closest-approach midpoint of rays (vectorised). Returns (points, ray gap)."""
    w0 = oa - ob
    a = np.sum(da * da, -1)
    b = np.sum(da * db, -1)
    c = np.sum(db * db, -1)
    d = np.sum(da * w0, -1)
    e = np.sum(db * w0, -1)
    den = a * c - b * b
    den = np.where(np.abs(den) < 1e-15, 1e-15, den)
    s = (b * e - c * d) / den
    t = (a * e - b * d) / den
    pa = oa + s[..., None] * da
    pb = ob + t[..., None] * db
    return 0.5 * (pa + pb), np.linalg.norm(pa - pb, axis=-1)


def triangulate_pixels(cam_a: CameraModel, uv_a: np.ndarray, cam_b: CameraModel, uv_b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    oa, da = cam_a.pixel_rays_world(np.atleast_2d(uv_a))
    ob, db = cam_b.pixel_rays_world(np.atleast_2d(uv_b))
    return midpoint_triangulate(oa, da, ob, db)


def rotation_angle_deg(R: np.ndarray) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


def tilt_from_gravity(g_unit: np.ndarray) -> tuple[float, float]:
    """(pitch, roll) in degrees of a gravity vector measured in the unit frame
    (unit x = beam / down-range, y = forward, z = up when level)."""
    g = np.asarray(g_unit, float)
    g = g / np.linalg.norm(g)
    return math.degrees(math.atan2(g[1], -g[2])), math.degrees(math.atan2(g[0], -g[2]))


# ------------------------------------------------------------ world alignment
def board_pose(cam: CameraModel, image: np.ndarray, board: BoardSpec) -> tuple[np.ndarray, np.ndarray, float, int]:
    """Board pose in the camera frame: (R_cb, t_cb, reprojection rms px, n corners)."""
    r = detect_board(image, board)
    if r is None:
        raise ValueError("calibration board not found")
    ok, rvec, tvec = cv2.solvePnP(r[0], r[1], cam.K, cam.dist)
    if not ok:
        raise ValueError("solvePnP failed")
    rvec, tvec = cv2.solvePnPRefineLM(r[0], r[1], cam.K, cam.dist, rvec, tvec)
    p, _ = cv2.projectPoints(r[0], rvec, tvec, cam.K, cam.dist)
    rms = float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - r[1]) ** 2, 1))))
    return cv2.Rodrigues(rvec)[0], tvec.ravel(), rms, len(r[0])


def align_world_from_board(intr0: CameraModel, intr1: CameraModel, stereo: StereoResult,
                           image0: np.ndarray, board: BoardSpec, ball_xy_on_board: Sequence[float],
                           board_thickness_m: float = 0.0, gravity_cam0: np.ndarray | None = None,
                           lights_cam0: np.ndarray | None = None,
                           meta: dict | None = None) -> RigCalibration:
    """Build the rig calibration in the contract frame from one view of the mat board.

    ``ball_xy_on_board``: board coordinates (m) of the point placed at the ball
    rest position. The board +x axis must point down the target line; its
    alignment error goes 1:1 into ``hla_deg``.
    ``gravity_cam0``: gravity direction in the cam0 frame (from the unit's tilt
    sensor rotated by the mechanical IMU->cam0 rotation); when given it
    defines "down", otherwise the board normal defines "up".
    ``lights_cam0``: LED positions in the cam0 frame (mechanical design).
    """
    R_cb, t_cb, rms, n = board_pose(intr0, image0, board)
    x_b, z_b = R_cb[:, 0], R_cb[:, 2]
    if np.dot(z_b, -t_cb) < 0:               # board normal toward the camera = up
        z_b = -z_b
    up = z_b
    if gravity_cam0 is not None:
        g = np.asarray(gravity_cam0, float)
        up = -g / np.linalg.norm(g)
    x = x_b - np.dot(x_b, up) * up
    x /= np.linalg.norm(x)
    y = np.cross(up, x)
    M = np.stack([x, y, up], axis=1)         # columns: world axes in the cam0 frame
    bx, by = ball_xy_on_board
    origin_c = R_cb @ np.array([bx, by, 0.0]) + t_cb + z_b * (BALL_RADIUS_M - board_thickness_m)
    # X_c0 = M X_w + origin_c
    c0 = intr0.with_pose(M, origin_c)
    c0.name = CAM_NAMES[0]
    c1 = intr1.with_pose(stereo.R @ M, stereo.R @ origin_c + stereo.T)
    c1.name = CAM_NAMES[1]
    lights = np.zeros((0, 3))
    if lights_cam0 is not None:
        lights = (np.asarray(lights_cam0, float).reshape(-1, 3) - origin_c) @ M
    m = {"created": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "align_rms_px": rms,
         "align_corners": n, "stereo_rms_px": stereo.rms_px, "up_from": "imu" if gravity_cam0 is not None else "board",
         "board_tilt_deg": math.degrees(math.acos(min(1.0, abs(float(np.dot(z_b, up))))))}
    if gravity_cam0 is not None:
        m["gravity_cam0"] = [float(v) for v in np.asarray(gravity_cam0, float)]
    m.update(meta or {})
    return RigCalibration({CAM_NAMES[0]: c0, CAM_NAMES[1]: c1}, lights, m)


# -------------------------------------------------------------------- check
@dataclass
class CheckReport:
    ok: bool
    moved: bool
    details: dict[str, Any]


def check_with_board(rig: RigCalibration, image0: np.ndarray, image1: np.ndarray, board: BoardSpec,
                     board_at_reference: bool = False, ball_xy_on_board: Sequence[float] = (0.0, 0.0),
                     board_thickness_m: float = 0.0, max_epipolar_px: float = 0.6,
                     max_shift_m: float = 0.002, max_rot_deg: float = 0.3) -> CheckReport:
    """Quick check with the board anywhere in view of both cameras.

    Stereo consistency (corner ray gap / epipolar error) reveals a camera that
    moved relative to the other. With ``board_at_reference`` (board back at its
    alignment position) the world pose is compared as well.
    """
    r0, r1 = detect_board(image0, board), detect_board(image1, board)
    if r0 is None or r1 is None:
        return CheckReport(False, False, {"error": "board not found in both cameras"})
    _, k0, k1 = np.intersect1d(r0[2], r1[2], return_indices=True)
    if len(k0) < 6:
        return CheckReport(False, False, {"error": "too few common corners"})
    c0, c1 = rig.pair
    X, gap = triangulate_pixels(c0, r0[1][k0], c1, r1[1][k1])
    e0 = np.linalg.norm(c0.project(X) - r0[1][k0], axis=1)
    e1 = np.linalg.norm(c1.project(X) - r1[1][k1], axis=1)
    epi = float(np.sqrt(np.mean(np.concatenate([e0, e1]) ** 2)))
    # rigid fit board -> world to check metric scale (baseline) as well
    obj = r0[0][k0]
    scale = float(np.linalg.norm(X - X.mean(0), axis=1).mean() / np.linalg.norm(obj - obj.mean(0), axis=1).mean())
    det: dict[str, Any] = {"n_corners": int(len(k0)), "stereo_reproj_rms_px": epi,
                           "ray_gap_mm_rms": float(np.sqrt(np.mean(gap ** 2)) * 1e3), "scale_error_pct": (scale - 1) * 100}
    moved = epi > max_epipolar_px or abs(scale - 1) > 0.004
    if board_at_reference:
        R_cb, t_cb, rms, _ = board_pose(c0.with_pose(np.eye(3), np.zeros(3)), image0, board)
        z_b = R_cb[:, 2] if np.dot(R_cb[:, 2], -t_cb) > 0 else -R_cb[:, 2]
        org = R_cb @ np.array([*ball_xy_on_board, 0.0]) + t_cb + z_b * (BALL_RADIUS_M - board_thickness_m)
        org_w = c0.R.T @ (org - c0.t)
        x_w = c0.R.T @ R_cb[:, 0]
        det.update({"origin_shift_mm": float(np.linalg.norm(org_w) * 1e3),
                    "target_line_yaw_deg": math.degrees(math.atan2(-x_w[1], x_w[0])),
                    "board_reproj_rms_px": rms})
        moved = moved or np.linalg.norm(org_w) > max_shift_m or abs(det["target_line_yaw_deg"]) > max_rot_deg
    return CheckReport(not moved, bool(moved), det)


def check_with_ball(rig: RigCalibration, ray0_w: np.ndarray, ray1_w: np.ndarray, dist0_m: float, dist1_m: float,
                    gravity_cam0: np.ndarray | None = None, max_gap_m: float = 0.0008,
                    max_height_m: float = 0.004, max_tilt_deg: float = 0.3) -> CheckReport:
    """Check without a board, using the ball at rest: the two centre rays must
    meet (relative camera pose), the triangulated distance must agree with the
    distance implied by the ball's apparent size (scale), the ball centre must
    sit at z = 0 (the unit was not moved relative to the mat) and the tilt
    sensor must read what it read at calibration time."""
    c0, c1 = rig.pair
    X, gap = midpoint_triangulate(c0.center, ray0_w, c1.center, ray1_w)
    d0, d1 = float(np.linalg.norm(X - c0.center)), float(np.linalg.norm(X - c1.center))
    det: dict[str, Any] = {"ray_gap_mm": float(gap) * 1e3, "ball_position_mm": (X * 1e3).round(2).tolist(),
                           "size_distance_error_pct": [100 * (dist0_m / d0 - 1), 100 * (dist1_m / d1 - 1)]}
    moved = bool(gap > max_gap_m or abs(X[2]) > max_height_m
                 or max(abs(dist0_m / d0 - 1), abs(dist1_m / d1 - 1)) > 0.03)
    g_ref = rig.meta.get("gravity_cam0")
    if gravity_cam0 is not None and g_ref is not None:
        a, b = np.asarray(gravity_cam0, float), np.asarray(g_ref, float)
        ang = math.degrees(math.acos(np.clip(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b), -1, 1)))
        det["tilt_change_deg"] = ang
        moved = moved or ang > max_tilt_deg
    return CheckReport(not moved, moved, det)
