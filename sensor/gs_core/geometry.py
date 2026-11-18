"""Camera model, projections and the coordinate conventions of docs/ARCHITECTURE.md section 3.

World frame: right-handed, origin at the ball centre at rest, +X down the target
line, +Y left of the target line (seen from behind the ball), +Z up.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

BALL_DIAMETER_M = 0.04267
BALL_RADIUS_M = BALL_DIAMETER_M / 2.0
RPM_TO_RADS = 2.0 * math.pi / 60.0


# --------------------------------------------------------------- camera model
@dataclass
class CameraModel:
    """Pinhole + OpenCV (k1,k2,p1,p2,k3) distortion, with world pose.

    Pixel coordinates are those of the delivered frame; when the sensor is
    ROI-cropped, use :meth:`with_roi` to shift the principal point.
    ``R``, ``t`` map world to camera: X_c = R @ X_w + t.
    """

    name: str
    K: np.ndarray
    dist: np.ndarray
    image_size: tuple[int, int]               # (width, height)
    R: np.ndarray = field(default_factory=lambda: np.eye(3))
    t: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self) -> None:
        self.K = np.asarray(self.K, dtype=np.float64).reshape(3, 3)
        d = np.zeros(5)
        dd = np.asarray(self.dist, dtype=np.float64).ravel()
        d[: min(5, dd.size)] = dd[:5]
        self.dist = d
        self.R = np.asarray(self.R, dtype=np.float64).reshape(3, 3)
        self.t = np.asarray(self.t, dtype=np.float64).reshape(3)
        self.image_size = (int(self.image_size[0]), int(self.image_size[1]))

    # ---------------------------------------------------------------- basics
    @property
    def fx(self) -> float:
        return float(self.K[0, 0])

    @property
    def fy(self) -> float:
        return float(self.K[1, 1])

    @property
    def center(self) -> np.ndarray:
        """Camera centre in world coordinates."""
        return -self.R.T @ self.t

    def with_roi(self, x0: int, y0: int, width: int, height: int) -> "CameraModel":
        K = self.K.copy()
        K[0, 2] -= x0
        K[1, 2] -= y0
        return CameraModel(self.name, K, self.dist.copy(), (width, height), self.R.copy(), self.t.copy())

    def with_pose(self, R: np.ndarray, t: np.ndarray) -> "CameraModel":
        return CameraModel(self.name, self.K.copy(), self.dist.copy(), self.image_size, R, t)

    def to_cam(self, Xw: np.ndarray) -> np.ndarray:
        return np.asarray(Xw, dtype=np.float64) @ self.R.T + self.t

    def to_world_dir(self, dc: np.ndarray) -> np.ndarray:
        return np.asarray(dc, dtype=np.float64) @ self.R

    # ------------------------------------------------------------ projection
    def distort_normalized(self, xy: np.ndarray) -> np.ndarray:
        k1, k2, p1, p2, k3 = self.dist
        x = xy[..., 0]
        y = xy[..., 1]
        r2 = x * x + y * y
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        return np.stack([xd, yd], axis=-1)

    def normalized_to_pixel(self, xy: np.ndarray) -> np.ndarray:
        d = self.distort_normalized(xy)
        u = self.K[0, 0] * d[..., 0] + self.K[0, 1] * d[..., 1] + self.K[0, 2]
        v = self.K[1, 1] * d[..., 1] + self.K[1, 2]
        return np.stack([u, v], axis=-1)

    def project_cam(self, Xc: np.ndarray) -> np.ndarray:
        """Project camera-frame points (..., 3) to pixels (..., 2)."""
        Xc = np.asarray(Xc)
        z = Xc[..., 2]
        xy = Xc[..., :2] / z[..., None]
        return self.normalized_to_pixel(xy)

    def project(self, Xw: np.ndarray) -> np.ndarray:
        """Project world points (..., 3) to pixels (..., 2)."""
        return self.project_cam(self.to_cam(Xw))

    def undistort_normalized(self, uv: np.ndarray) -> np.ndarray:
        """Pixels (..., 2) -> ideal normalized image coordinates (..., 2)."""
        uv = np.asarray(uv, dtype=np.float64)
        shp = uv.shape
        pts = uv.reshape(-1, 1, 2)
        if not np.any(self.dist):
            out = (pts.reshape(-1, 2) - self.K[:2, 2]) / np.array([self.fx, self.fy])
            return out.reshape(shp)
        crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 1e-12)
        try:
            und = cv2.undistortPoints(pts, self.K, self.dist, None, None, criteria=crit)
        except TypeError:   # older/newer binding without the criteria kwarg
            und = cv2.undistortPoints(pts, self.K, self.dist)
        und = und.reshape(-1, 2)
        # one Newton polish step for accuracy (OpenCV's fixed-point iteration is coarse)
        und = self._polish_undistort(pts.reshape(-1, 2), und)
        return und.reshape(shp)

    def _polish_undistort(self, uv: np.ndarray, xy: np.ndarray, iters: int = 2) -> np.ndarray:
        for _ in range(iters):
            p = self.normalized_to_pixel(xy)
            err = uv - p
            eps = 1e-6
            jx = (self.normalized_to_pixel(xy + [eps, 0]) - p) / eps
            jy = (self.normalized_to_pixel(xy + [0, eps]) - p) / eps
            a, b, c, d = jx[:, 0], jy[:, 0], jx[:, 1], jy[:, 1]
            det = a * d - b * c
            det = np.where(np.abs(det) < 1e-12, 1e-12, det)
            dx = (d * err[:, 0] - b * err[:, 1]) / det
            dy = (-c * err[:, 0] + a * err[:, 1]) / det
            xy = xy + np.stack([dx, dy], axis=1)
        return xy

    def pixel_rays_cam(self, uv: np.ndarray) -> np.ndarray:
        """Unit ray directions in the camera frame for pixels (..., 2)."""
        xy = self.undistort_normalized(uv)
        d = np.concatenate([xy, np.ones(xy.shape[:-1] + (1,))], axis=-1)
        return d / np.linalg.norm(d, axis=-1, keepdims=True)

    def pixel_rays_world(self, uv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(origin (3,), unit directions (..., 3)) in world frame."""
        return self.center, self.to_world_dir(self.pixel_rays_cam(uv))

    # ---------------------------------------------------------------- serial
    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "K": self.K.tolist(),
            "dist": self.dist.tolist(),
            "image_size": list(self.image_size),
            "R": self.R.tolist(),
            "t": self.t.tolist(),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "CameraModel":
        return cls(d["name"], np.array(d["K"]), np.array(d["dist"]), tuple(d["image_size"]),
                   np.array(d["R"]), np.array(d["t"]))


def look_at(eye: np.ndarray, target: np.ndarray, up: np.ndarray = np.array([0.0, 0.0, 1.0]),
            roll_deg: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """World->camera (R, t) for a camera at ``eye`` looking at ``target``.

    Camera axes follow OpenCV: +z forward, +x right, +y down. ``roll_deg``
    rotates the sensor about the optical axis (90 = portrait mounting with the
    sensor's +x pointing world-up).
    """
    eye = np.asarray(eye, float)
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    Rcw = np.stack([x, y, z])       # rows: camera axes in world
    if roll_deg:
        a = math.radians(roll_deg)
        Rz = np.array([[math.cos(a), math.sin(a), 0], [-math.sin(a), math.cos(a), 0], [0, 0, 1]])
        Rcw = Rz @ Rcw
    t = -Rcw @ eye
    return Rcw, t


# --------------------------------------------------------- launch conventions
def launch_from_velocity(v: np.ndarray) -> tuple[float, float, float]:
    """World velocity -> (speed m/s, vla deg, hla deg). hla positive = right (-Y)."""
    v = np.asarray(v, float)
    speed = float(np.linalg.norm(v))
    vla = math.degrees(math.atan2(v[2], math.hypot(v[0], v[1])))
    hla = math.degrees(math.atan2(-v[1], v[0]))
    return speed, vla, hla


def velocity_from_launch(speed: float, vla_deg: float, hla_deg: float) -> np.ndarray:
    a, b = math.radians(vla_deg), math.radians(hla_deg)
    return speed * np.array([math.cos(a) * math.cos(b), -math.cos(a) * math.sin(b), math.sin(a)])


def spin_frame(v_dir: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Orthonormal (forward, left, up') frame attached to the launch direction."""
    f = np.asarray(v_dir, float) / np.linalg.norm(v_dir)
    left = np.cross([0.0, 0.0, 1.0], f)
    n = np.linalg.norm(left)
    left = np.array([0.0, 1.0, 0.0]) if n < 1e-9 else left / n
    up = np.cross(f, left)
    return f, left, up


def spin_vector(total_rpm: float, axis_deg: float, v_dir: np.ndarray) -> np.ndarray:
    """Angular velocity (rad/s, world) for a spin of ``total_rpm`` with the given axis tilt.

    Pure backspin (axis 0) points toward -left (so the top of the ball moves
    backwards and the Magnus force is up). Positive axis tilt => Magnus force
    toward -Y (ball curves right), per the contract.
    """
    _, left, up = spin_frame(v_dir)
    a = math.radians(axis_deg)
    return total_rpm * RPM_TO_RADS * (-math.cos(a) * left - math.sin(a) * up)


def spin_from_vector(omega: np.ndarray, v_dir: np.ndarray) -> tuple[float, float]:
    """Angular velocity (rad/s, world) -> (total rpm, spin axis deg).

    The axis tilt is measured on the component perpendicular to the flight
    direction (rifle spin does not curve the ball); total is |omega|.
    """
    _, left, up = spin_frame(v_dir)
    total = float(np.linalg.norm(omega)) / RPM_TO_RADS
    axis = math.degrees(math.atan2(-float(np.dot(omega, up)), -float(np.dot(omega, left))))
    return total, axis


def rotation_matrix(rotvec: np.ndarray) -> np.ndarray:
    """Rodrigues rotation matrix (vectorised over leading dims)."""
    rv = np.asarray(rotvec, dtype=np.float64)
    theta = np.linalg.norm(rv, axis=-1, keepdims=True)
    small = theta < 1e-12
    k = np.where(small, 0.0, rv / np.where(small, 1.0, theta))
    kx, ky, kz = k[..., 0], k[..., 1], k[..., 2]
    z = np.zeros_like(kx)
    Kx = np.stack([np.stack([z, -kz, ky], -1), np.stack([kz, z, -kx], -1), np.stack([-ky, kx, z], -1)], -2)
    th = theta[..., None]
    eye = np.broadcast_to(np.eye(3), Kx.shape)
    return eye + np.sin(th) * Kx + (1 - np.cos(th)) * (Kx @ Kx)
