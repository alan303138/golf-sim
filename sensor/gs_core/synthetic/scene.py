"""Synthetic rig geometry, ground-truth shots, ball flight and club motion."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..calibration import CAM_NAMES, RigCalibration
from ..geometry import (BALL_RADIUS_M, CameraModel, look_at, rotation_matrix, spin_vector,
                        velocity_from_launch)
from ..physics import AeroModel, integrate


# ---------------------------------------------------------------------- rig
@dataclass
class RigSpec:
    """Nominal side-on unit (hardware/enclosure nominal pose). The unit stands on
    the -Y side of the ball (opposite a right-handed golfer), looking toward +Y."""

    image_size: tuple[int, int] = (1456, 1088)
    pixel_m: float = 3.45e-6
    focal_m: float = 0.008
    baseline_m: float = 0.160
    ball_dist_m: float = 0.475          # horizontal, lens pupil -> ball
    lens_height_m: float = 0.255        # optical axis above the mat
    pitch_deg: float = 17.0
    aim_x_m: float = 0.118              # mean pupil is this far down-range of the ball
    toe_in_downrange_deg: float = 6.0   # camera on the +X side turns toward -X
    toe_in_uprange_deg: float = 9.0     # camera on the -X side turns toward +X
    led_offset_m: float = 0.140         # two LED boards, +- this from the pair centreline
    k1: float = -0.08                   # mild barrel distortion of an 8 mm C-mount lens
    k2: float = 0.05


def make_rig(spec: RigSpec | None = None) -> RigCalibration:
    s = spec or RigSpec()
    f = s.focal_m / s.pixel_m
    W, H = s.image_size
    p = math.radians(s.pitch_deg)
    fwd0 = np.array([0.0, math.cos(p), -math.sin(p)])
    up_b = np.array([0.0, math.sin(p), math.cos(p)])
    xh = np.array([1.0, 0.0, 0.0])
    zc = s.lens_height_m - BALL_RADIUS_M
    cams = {}
    # cam0 = up-range (-X side), cam1 = down-range (+X side)
    for name, side, toe in ((CAM_NAMES[0], -1, s.toe_in_uprange_deg), (CAM_NAMES[1], 1, s.toe_in_downrange_deg)):
        eye = np.array([s.aim_x_m + side * s.baseline_m / 2, -s.ball_dist_m, zc])
        a = math.radians(toe)
        fwd = math.cos(a) * fwd0 - side * math.sin(a) * xh
        R, t = look_at(eye, eye + fwd, up=up_b)
        K = np.array([[f, 0, W / 2 - 0.5 + 3.0 * side], [0, f, H / 2 - 0.5 - 2.0 * side], [0, 0, 1]])
        cams[name] = CameraModel(name, K, np.array([s.k1, s.k2, 0.0003 * side, -0.0002, 0.0]), (W, H), R, t)
    lights = np.array([[s.aim_x_m + sg * s.led_offset_m, -s.ball_dist_m, zc] for sg in (-1, 1)])
    return RigCalibration(cams, lights, {"synthetic": True, "led_axis": fwd0.tolist()})


def perturbed_rig(rig: RigCalibration, rng: np.random.Generator, scale: float = 1.0) -> RigCalibration:
    """A rig as a real calibration would deliver it: the truth plus errors of
    the size a careful ChArUco calibration and a hand-placed mat board give.

    1-sigma (x ``scale``): focal 0.1 %, principal point 0.7 px, k1 1.5 %,
    relative camera rotation 0.015 deg, baseline 0.08 %, world alignment
    0.2 deg yaw / 0.1 deg pitch+roll, origin 1 mm, LED positions 5 mm.
    """
    c0, c1 = rig.pair

    def intr(c: CameraModel) -> CameraModel:
        K = c.K.copy()
        fs = 1 + rng.normal(0, 0.001) * scale
        K[0, 0] *= fs
        K[1, 1] *= fs
        K[:2, 2] += rng.normal(0, 0.7, 2) * scale
        d = c.dist.copy()
        d[0] *= 1 + rng.normal(0, 0.015) * scale
        return CameraModel(c.name, K, d, c.image_size, c.R.copy(), c.t.copy())

    a, b = intr(c0), intr(c1)
    # relative pose error: rotate/shift cam1 about cam0
    dR = rotation_matrix(np.radians(rng.normal(0, 0.015, 3) * scale))
    base = c1.center - c0.center
    ctr1 = c0.center + base * (1 + rng.normal(0, 0.0008) * scale)
    b = b.with_pose(dR @ b.R, -(dR @ b.R) @ ctr1)
    # world alignment error: common rigid transform X_w' = A X_w + o
    ang = np.radians(np.array([rng.normal(0, 0.1), rng.normal(0, 0.1), rng.normal(0, 0.2)]) * scale)
    A = rotation_matrix(ang)
    o = rng.normal(0, 0.001, 3) * scale
    out = {}
    for c in (a, b):
        # X_c = R X_w + t = R A^T (X_w' - o) + t
        out[c.name] = c.with_pose(c.R @ A.T, c.t - c.R @ A.T @ o)
    lights = rig.lights @ A.T + o + rng.normal(0, 0.005, rig.lights.shape) * scale
    return RigCalibration(out, lights, dict(rig.meta, perturbed=True))


# -------------------------------------------------------------------- shots
@dataclass
class ShotTruth:
    speed_mps: float
    vla_deg: float
    hla_deg: float = 0.0
    total_spin_rpm: float = 0.0
    spin_axis_deg: float = 0.0
    rolling: bool = False               # putt: ball stays on the mat and decelerates
    club_speed_mps: float | None = None
    club: bool = True                   # render a club head passing through
    label: str = ""

    @property
    def velocity(self) -> np.ndarray:
        return velocity_from_launch(self.speed_mps, self.vla_deg, self.hla_deg)

    @property
    def omega(self) -> np.ndarray:
        if self.rolling:                # pure rolling: top of the ball moves forward
            v = self.velocity
            return np.cross([0.0, 0.0, 1.0], v) / BALL_RADIUS_M
        return spin_vector(self.total_spin_rpm, self.spin_axis_deg, self.velocity)

    @property
    def club_speed(self) -> float:
        if self.club_speed_mps:
            return self.club_speed_mps
        smash = 1.48 if self.vla_deg < 14 else (1.35 if self.vla_deg < 22 else 1.18)
        return self.speed_mps / (1.0 if self.rolling else smash)


PRESETS: dict[str, ShotTruth] = {
    "driver": ShotTruth(75.0, 11.0, 0.0, 2700.0, 0.0, label="driver"),
    "iron7": ShotTruth(54.0, 16.0, 0.0, 7000.0, 0.0, label="iron7"),
    "wedge": ShotTruth(40.0, 28.0, 0.0, 9500.0, 0.0, label="wedge"),
    "chip": ShotTruth(15.0, 30.0, 0.0, 3500.0, 0.0, label="chip"),
    "putt": ShotTruth(3.0, 0.0, 0.0, 0.0, 0.0, rolling=True, label="putt"),
}


def random_shot(rng: np.random.Generator) -> ShotTruth:
    """A random but realistic shot from a mix of clubs."""
    kind = str(rng.choice(["driver", "wood", "mid_iron", "short_iron", "wedge", "chip", "putt"],
                          p=[0.22, 0.1, 0.26, 0.17, 0.13, 0.07, 0.05]))
    if kind == "putt":
        return ShotTruth(float(rng.uniform(1.0, 5.0)), 0.0, float(rng.normal(0, 1.5)), rolling=True, label=kind)
    if kind == "chip":
        return ShotTruth(float(rng.uniform(8, 20)), float(rng.uniform(20, 38)), float(rng.normal(0, 3)),
                         float(rng.uniform(2000, 5000)), float(rng.normal(0, 6)), label=kind)
    base = {"driver": (68, 11.5, 2700), "wood": (62, 12.5, 3800), "mid_iron": (54, 16.0, 6500),
            "short_iron": (47, 21.0, 8000), "wedge": (38, 28.0, 9300)}[kind]
    return ShotTruth(float(base[0] * rng.uniform(0.85, 1.12)), float(base[1] + rng.normal(0, 2.0)),
                     float(rng.normal(0, 3.0)), float(base[2] * rng.uniform(0.8, 1.2)),
                     float(rng.normal(0, 10.0)), label=kind)


def random_orientation(rng: np.random.Generator) -> np.ndarray:
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


ROLL_DECEL = 0.6        # m/s^2, rolling resistance of a putting mat


class Flight:
    """Ball state over time; at rest at ``rest`` until ``t_impact`` (seconds)."""

    def __init__(self, shot: ShotTruth, t_impact: float, rest: np.ndarray | None = None,
                 orientation0: np.ndarray | None = None, aero: AeroModel | None = None) -> None:
        self.shot = shot
        self.t_impact = t_impact
        self.rest = np.zeros(3) if rest is None else np.asarray(rest, float)
        self.R0 = np.eye(3) if orientation0 is None else orientation0
        self.omega = shot.omega
        self._v0 = shot.velocity
        if not shot.rolling:
            dt = 2.5e-4
            self._dt = dt
            self._t = np.arange(0.0, 0.12 + dt, dt)
            self._p, self._v = integrate(np.zeros(3), self._v0, self.omega, self._t, aero or AeroModel(), dt=dt)

    def position(self, t: float) -> np.ndarray:
        tau = t - self.t_impact
        if tau <= 0:
            return self.rest.copy()
        if self.shot.rolling:
            sp = self.shot.speed_mps
            tau = min(tau, sp / ROLL_DECEL)
            return self.rest + self._v0 / sp * (sp * tau - 0.5 * ROLL_DECEL * tau * tau)
        x = min(tau, self._t[-1] - 1e-9)
        i = min(int(x / self._dt), len(self._t) - 2)
        s = (x - self._t[i]) / self._dt
        h = self._dt
        p = ((2 * s ** 3 - 3 * s ** 2 + 1) * self._p[i] + (s ** 3 - 2 * s ** 2 + s) * h * self._v[i]
             + (-2 * s ** 3 + 3 * s ** 2) * self._p[i + 1] + (s ** 3 - s ** 2) * h * self._v[i + 1])
        return self.rest + p

    def orientation(self, t: float) -> np.ndarray:
        tau = max(0.0, t - self.t_impact)
        return rotation_matrix(self.omega * tau) @ self.R0


@dataclass
class ClubMotion:
    """Ellipsoidal club head (semi-axes along X, Y, Z) passing through impact."""

    speed: float
    t_impact: float
    rest: np.ndarray = field(default_factory=lambda: np.zeros(3))
    attack_deg: float = -2.0
    semi_axes: tuple[float, float, float] = (0.028, 0.050, 0.022)
    albedo: float = 0.30

    def center(self, t: float) -> np.ndarray:
        tau = t - self.t_impact
        a = math.radians(self.attack_deg)
        d_in = np.array([math.cos(a), 0.0, math.sin(a)])
        c_imp = self.rest + np.array([-BALL_RADIUS_M - self.semi_axes[0], 0.0, 0.0])
        if tau <= 0:
            return c_imp + d_in * self.speed * tau
        # after impact: slower, swinging up and around toward the golfer (+Y)
        d_out = np.array([0.90, 0.25, 0.36])
        return c_imp + d_out / np.linalg.norm(d_out) * 0.72 * self.speed * tau

    def shaft(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        c = self.center(t)
        heel = c + np.array([0.0, self.semi_axes[1] * 0.8, 0.012])
        return heel, heel + np.array([0.04, 0.40, 0.70])
