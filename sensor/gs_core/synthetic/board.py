"""Render a ChArUco calibration board seen by a (distorted) camera."""
from __future__ import annotations

import cv2
import numpy as np

from ..calibration import CharucoSpec
from ..geometry import CameraModel


def board_texture(spec: CharucoSpec, px_per_m: float = 4000.0) -> np.ndarray:
    w = int(spec.squares_x * spec.square_m * px_per_m)
    h = int(spec.squares_y * spec.square_m * px_per_m)
    return spec.board().generateImage((w, h), marginSize=0, borderBits=1)


def render_board(cam: CameraModel, spec: CharucoSpec, R_wb: np.ndarray, t_wb: np.ndarray, tex: np.ndarray,
                 noise: float = 2.0, rng: np.random.Generator | None = None, lit: float = 200.0,
                 dark: float = 20.0) -> np.ndarray:
    """Board pose: X_w = R_wb @ X_b + t_wb (board in its z=0 plane, origin at a corner).
    Returns an 8-bit image as the camera sees it (with lens distortion)."""
    W, H = cam.image_size
    U, V = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    xy = cam.undistort_normalized(np.stack([U, V], -1))
    d = np.concatenate([xy, np.ones(xy.shape[:2] + (1,))], -1) @ cam.R          # world directions
    o = cam.center
    n = R_wb[:, 2]
    t = ((t_wb - o) @ n) / np.where(np.abs(d @ n) < 1e-12, 1e-12, d @ n)
    X = o + d * t[..., None]
    b = (X - t_wb) @ R_wb                                                    # board coordinates
    ppm = tex.shape[1] / (spec.squares_x * spec.square_m)
    mx = (b[..., 0] * ppm - 0.5).astype(np.float32)
    # generateImage draws row 0 at the top; ChArUco object y runs from that edge downward
    my = (b[..., 1] * ppm - 0.5).astype(np.float32)
    img = cv2.remap(tex, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                    borderValue=128)
    img = np.where(t > 0, img, 0).astype(np.float32)
    img = dark + (lit - dark) * img / 255.0
    img = cv2.GaussianBlur(img, (0, 0), 0.8)
    rng = rng or np.random.default_rng(0)
    img += rng.normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)
