"""Synthetic golf ball surface: dimples (normal perturbation) and prints (albedo).

The surface is baked into a cube map holding, for every body-frame direction,
the perturbed unit normal and the albedo. The dimple layout is a class-I
geodesic icosahedron (362 dimples for frequency 6), i.e. it has the high
(icosahedral) symmetry of real dimple patterns - deliberately a hard case for
markerless spin, because a symmetric pattern alone cannot distinguish
rotations that differ by a symmetry of the ball.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial import cKDTree

from ..geometry import BALL_RADIUS_M


def geodesic_points(freq: int) -> np.ndarray:
    """Vertices of a class-I geodesic icosahedron, unit vectors (10 f^2 + 2 points)."""
    phi = (1 + math.sqrt(5)) / 2
    v = np.array([[-1, phi, 0], [1, phi, 0], [-1, -phi, 0], [1, -phi, 0],
                  [0, -1, phi], [0, 1, phi], [0, -1, -phi], [0, 1, -phi],
                  [phi, 0, -1], [phi, 0, 1], [-phi, 0, -1], [-phi, 0, 1]], float)
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2),
             (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5),
             (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    pts = []
    for a, b, c in faces:
        for i in range(freq + 1):
            for j in range(freq + 1 - i):
                k = freq - i - j
                # slerp-like warp (tan) makes the spacing more uniform than plain normalisation
                p = v[a] * i + v[b] * j + v[c] * k
                pts.append(p / np.linalg.norm(p))
    pts = np.array(pts)
    key = np.round(pts * 1e6).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return pts[np.sort(idx)]


@dataclass
class BallSpec:
    dimples: bool = True
    dimple_freq: int = 6               # 362 dimples
    dimple_depth_m: float = 0.00020
    dimple_fill: float = 0.47          # dimple angular radius / nearest-neighbour distance
    albedo: float = 0.85               # at 850 nm
    marking: str = "logo"              # "none" | "logo" | "line" | "dots"
    mark_albedo: float = 0.12
    line_width_m: float = 0.003
    n_dots: int = 14
    dot_radius_m: float = 0.0025
    cube_n: int = 320
    seed: int = 1


class BallModel:
    """Baked surface: ``lookup(body_dirs) -> (normals (N,3), albedo (N,))`` in the body frame."""

    def __init__(self, spec: BallSpec | None = None) -> None:
        self.spec = spec or BallSpec()
        self._bake()

    # ------------------------------------------------------------------
    def _bake(self) -> None:
        s = self.spec
        n = s.cube_n
        g = (np.arange(n) + 0.5) / n * 2 - 1
        a, b = np.meshgrid(g, g, indexing="ij")
        one = np.ones_like(a)
        dirs = []
        for axis in range(3):
            for sign in (1.0, -1.0):
                d = [None, None, None]
                d[axis] = sign * one
                d[(axis + 1) % 3] = a
                d[(axis + 2) % 3] = b
                dirs.append(np.stack(d, -1))
        dirs = np.stack(dirs).reshape(-1, 3)
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        normals = dirs.copy()
        if s.dimples:
            centres = geodesic_points(s.dimple_freq)
            tree = cKDTree(centres)
            dd, _ = tree.query(centres, k=2)
            rad = 2 * np.arcsin(dd[:, 1] / 2) * s.dimple_fill          # angular radius per dimple
            _, idx = tree.query(dirs)
            c = centres[idx]
            cosang = np.clip(np.sum(c * dirs, 1), -1, 1)
            psi = np.arccos(cosang)
            x = psi / rad[idx]
            # slope of a parabolic bowl, rolled off smoothly at the rim
            smax = 2 * s.dimple_depth_m / (rad[idx] * BALL_RADIUS_M)
            roll = np.clip((1.0 - x) / 0.2, 0, 1)
            slope = smax * x * roll * roll * (3 - 2 * roll) * (x < 1)
            # unit tangent pointing from the dimple centre outward
            tang = dirs * cosang[:, None] - c
            tn = np.linalg.norm(tang, axis=1, keepdims=True)
            tang = np.where(tn > 1e-9, tang / np.maximum(tn, 1e-9), 0)
            normals = dirs - slope[:, None] * tang           # bowl: normals lean toward the centre
            normals /= np.linalg.norm(normals, axis=1, keepdims=True)
            self.dimple_centres = centres
        else:
            self.dimple_centres = np.zeros((0, 3))
        alb = np.full(len(dirs), s.albedo)
        rng = np.random.default_rng(s.seed)
        if s.marking == "logo":
            alb = self._logo(dirs, alb, np.array([0.3, -0.9, 0.3]), "GS-1", 0.016)
            alb = self._logo(dirs, alb, np.array([-0.5, 0.7, -0.5]), "4", 0.007)
        elif s.marking == "line":
            ax = np.array([0.2, 0.1, 0.97])
            ax /= np.linalg.norm(ax)
            alb = np.where(np.abs(dirs @ ax) < s.line_width_m / 2 / BALL_RADIUS_M, s.mark_albedo, alb)
            alb = self._logo(dirs, alb, np.array([0.9, -0.3, 0.3]), "GS-1", 0.016)
        elif s.marking == "dots":
            q = rng.normal(size=(s.n_dots, 3))
            q /= np.linalg.norm(q, axis=1, keepdims=True)
            dmin = np.arccos(np.clip(dirs @ q.T, -1, 1)).min(axis=1)
            alb = np.where(dmin < s.dot_radius_m / BALL_RADIUS_M, s.mark_albedo, alb)
        self._tex = np.concatenate([normals, alb[:, None]], 1).astype(np.float32).reshape(6, n, n, 4)

    def _logo(self, dirs: np.ndarray, alb: np.ndarray, centre: np.ndarray, text: str, width_m: float) -> np.ndarray:
        c = centre / np.linalg.norm(centre)
        e1 = np.cross([0.0, 0.0, 1.0], c)
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(c, e1)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 2.0, 5)
        img = np.zeros((th + 30, tw + 30), np.uint8)
        cv2.putText(img, text, (15, th + 12), cv2.FONT_HERSHEY_DUPLEX, 2.0, 255, 5, cv2.LINE_AA)
        scale = width_m / BALL_RADIUS_M / img.shape[1]          # radians per logo pixel
        z = dirs @ c
        front = z > 0.5
        u = (dirs @ e1) / np.maximum(z, 1e-6) / scale + img.shape[1] / 2
        v = -(dirs @ e2) / np.maximum(z, 1e-6) / scale + img.shape[0] / 2
        ok = front & (u >= 0) & (u < img.shape[1] - 1) & (v >= 0) & (v < img.shape[0] - 1)
        ink = np.zeros(len(dirs))
        ink[ok] = img[v[ok].astype(int), u[ok].astype(int)] / 255.0
        return alb * (1 - ink) + self.spec.mark_albedo * ink

    # ------------------------------------------------------------------
    def lookup(self, b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        n = self.spec.cube_n
        ab = np.abs(b)
        axis = np.argmax(ab, axis=1)
        rows = np.arange(len(b))
        major = b[rows, axis]
        face = axis * 2 + (major < 0)
        inv = 1.0 / np.maximum(ab[rows, axis], 1e-12)
        u = b[rows, (axis + 1) % 3] * inv
        v = b[rows, (axis + 2) % 3] * inv
        iu = np.clip(((u + 1) * 0.5 * n).astype(np.intp), 0, n - 1)
        iv = np.clip(((v + 1) * 0.5 * n).astype(np.intp), 0, n - 1)
        t = self._tex[face, iu, iv]
        return t[:, :3].astype(np.float64), t[:, 3].astype(np.float64)
