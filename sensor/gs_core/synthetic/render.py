"""Multi-exposure frame renderer for one camera.

Every strobe pulse adds one image of the scene (additive exposure): the ball
at its position/orientation at that instant, ray-traced with dimple normals
and prints, Lambertian + weak specular shading from the LED boards, motion
blur by temporal supersampling over the pulse width, a faintly lit mat, an
optional club head. The sum (in electrons) gets lens blur, photon shot noise,
read noise, gain, black level and saturation.

Not modelled: cast shadows on the mat, inter-reflection between mat and ball,
sub-surface scattering / real BRDF of a ball cover, lens vignetting, defocus.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import cv2
import numpy as np

from ..geometry import BALL_RADIUS_M, CameraModel
from .ball import BallModel
from .scene import ClubMotion


@dataclass
class SensorSpec:
    ball_electrons: float = 450.0       # e- per pixel per 10 us pulse, white ball facing the unit
    ref_pulse_us: float = 10.0
    full_scale: int = 1023              # 10-bit raw
    black_level: float = 60.0
    gain_dn_per_e: float = 0.6          # analogue gain chosen so ~1600 e- saturates
    read_noise_e: float = 3.0
    psf_sigma_px: float = 0.7
    mat_albedo: float = 0.04            # dark, IR-absorbing mat
    mat_texture: float = 0.35
    specular: float = 0.25
    spec_exp: float = 60.0
    ambient: float = 0.03               # diffuse fill (inter-reflection), fraction of direct
    supersample: int = 2
    max_time_samples: int = 3


BallState = Callable[[float], "tuple[np.ndarray, np.ndarray] | None"]


class Renderer:
    def __init__(self, cam: CameraModel, lights: np.ndarray, led_axis: np.ndarray, ball: BallModel,
                 sensor: SensorSpec | None = None, seed: int = 0) -> None:
        self.cam = cam
        self.lights = np.asarray(lights, float).reshape(-1, 3)
        self.led_axis = np.asarray(led_axis, float) / np.linalg.norm(led_axis)
        self.ball = ball
        self.s = sensor or SensorSpec()
        self.rng = np.random.default_rng(seed)
        W, H = cam.image_size
        U, V = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
        self._xy = cam.undistort_normalized(np.stack([U, V], -1))
        self._gx = np.gradient(self._xy, axis=1)
        self._gy = np.gradient(self._xy, axis=0)
        # radiometric scale: white facet at the origin facing the unit centre
        n_ref = (self.lights.mean(0)) / np.linalg.norm(self.lights.mean(0))
        e_ref = self._irradiance(np.zeros((1, 3)), n_ref[None, :])[0]
        self._scale = self.s.ball_electrons / (ball.spec.albedo * e_ref)
        self.mat = self._make_mat()

    # ---------------------------------------------------------------- light
    def _irradiance(self, X: np.ndarray, n: np.ndarray, return_spec_for: np.ndarray | None = None):
        """Sum over LEDs of E_k * max(n.l_k, 0) (arbitrary units), optionally with specular."""
        tot = np.zeros(len(X))
        spec = np.zeros(len(X))
        for L in self.lights:
            lv = L - X
            d2 = np.sum(lv * lv, 1)
            l = lv / np.sqrt(d2)[:, None]
            emit = np.clip(-(l @ self.led_axis), 0, None)        # Lambertian emitter
            E = emit / d2
            ndl = np.clip(np.sum(n * l, 1), 0, None)
            tot += E * ndl
            if return_spec_for is not None:
                h = l - return_spec_for
                h /= np.linalg.norm(h, axis=1, keepdims=True)
                spec += E * (ndl > 0) * np.clip(np.sum(n * h, 1), 0, None) ** self.s.spec_exp
        if return_spec_for is not None:
            return tot, spec
        return tot

    def _make_mat(self) -> np.ndarray:
        """Electrons per reference pulse from the mat plane z = -R, per pixel."""
        W, H = self.cam.image_size
        d = np.concatenate([self._xy[::4, ::4], np.ones(self._xy[::4, ::4].shape[:2] + (1,))], -1).reshape(-1, 3)
        dw = d @ self.cam.R
        o = self.cam.center
        tt = (-BALL_RADIUS_M - o[2]) / np.where(np.abs(dw[:, 2]) < 1e-9, -1e-9, dw[:, 2])
        X = o + dw * tt[:, None]
        ok = (tt > 0) & (np.abs(X[:, 0] - 0.1) < 0.6) & (X[:, 1] < 0.5) & (X[:, 1] > -0.4)
        e = np.where(ok, self._irradiance(X, np.tile([0.0, 0.0, 1.0], (len(X), 1))), 0.0)
        small = (e * self._scale * self.s.mat_albedo).reshape(self._xy[::4, ::4].shape[:2]).astype(np.float32)
        mat = cv2.resize(small, (W, H), interpolation=cv2.INTER_LINEAR)
        rng = np.random.default_rng(1234 + hash(self.cam.name) % 1000)
        tex = cv2.GaussianBlur(rng.normal(0, 1, (H, W)).astype(np.float32), (0, 0), 1.5)
        tex /= tex.std()
        return (mat * np.clip(1 + self.s.mat_texture * tex, 0.1, None)).astype(np.float32)

    # ----------------------------------------------------------------- rays
    def _rays(self, box: tuple[int, int, int, int], ss: int) -> np.ndarray:
        x0, y0, x1, y1 = box
        offs = (np.arange(ss) + 0.5) / ss - 0.5
        c = self._xy[y0:y1, x0:x1]
        dx, dy = self._gx[y0:y1, x0:x1], self._gy[y0:y1, x0:x1]
        sub = (c[:, None, :, None, :] + offs[None, :, None, None, None] * dy[:, None, :, None, :]
               + offs[None, None, None, :, None] * dx[:, None, :, None, :]).reshape(-1, 2)
        d = np.concatenate([sub, np.ones((len(sub), 1))], 1)
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        return d @ self.cam.R                       # world directions

    def _bbox(self, pts_w: np.ndarray, pad_m: float) -> tuple[int, int, int, int] | None:
        cc = self.cam.to_cam(pts_w)
        if np.any(cc[:, 2] < 0.1):
            return None
        uv = self.cam.project_cam(cc)
        r = self.cam.fx * pad_m / cc[:, 2].min() * 1.12 + 4
        W, H = self.cam.image_size
        x0, y0 = int(math.floor(uv[:, 0].min() - r)), int(math.floor(uv[:, 1].min() - r))
        x1, y1 = int(math.ceil(uv[:, 0].max() + r)) + 1, int(math.ceil(uv[:, 1].max() + r)) + 1
        x0, y0, x1, y1 = max(0, x0), max(0, y0), min(W, x1), min(H, y1)
        if x1 <= x0 or y1 <= y0:
            return None
        return x0, y0, x1, y1

    def _down(self, a: np.ndarray, box: tuple[int, int, int, int], ss: int) -> np.ndarray:
        x0, y0, x1, y1 = box
        return a.reshape(y1 - y0, ss, x1 - x0, ss).mean(axis=(1, 3))

    # ----------------------------------------------------------------- ball
    def _ball_layer(self, box, states: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, np.ndarray]:
        """(radiance e-/ref pulse, coverage) of the ball averaged over the given sub-instants."""
        ss = self.s.supersample
        d = self._rays(box, ss)
        o = self.cam.center
        rad = np.zeros(len(d))
        cov = np.zeros(len(d))
        for c, Rb in states:
            oc = c - o
            b = d @ oc
            disc = b * b - (oc @ oc - BALL_RADIUS_M ** 2)
            hit = disc > 0
            if not np.any(hit):
                continue
            dh = d[hit]
            X = o + dh * (b[hit] - np.sqrt(disc[hit]))[:, None]
            n0 = (X - c) / BALL_RADIUS_M
            nb, alb = self.ball.lookup(n0 @ Rb)
            n = nb @ Rb.T
            diff, spec = self._irradiance(X, n, return_spec_for=dh)
            amb = self.s.ambient * self._irradiance(X, n0)          # crude fill light
            val = self._scale * (alb * (diff + amb) + self.s.specular * spec)
            rad[hit] += val
            cov[hit] += 1.0
        k = len(states)
        return self._down(rad / k, box, ss), self._down(cov / k, box, ss)

    def _club_layer(self, box, club: ClubMotion, t: float) -> tuple[np.ndarray, np.ndarray]:
        d = self._rays(box, 1)
        o = self.cam.center
        s = np.array(club.semi_axes)
        c = club.center(t)
        oo = (o - c) / s
        dd = d / s
        a = np.sum(dd * dd, 1)
        b = dd @ oo
        disc = b * b - a * (oo @ oo - 1.0)
        hit = disc > 0
        rad = np.zeros(len(d))
        if np.any(hit):
            tt = (-b[hit] - np.sqrt(disc[hit])) / a[hit]
            X = o + d[hit] * tt[:, None]
            n = (X - c) / (s * s)
            n /= np.linalg.norm(n, axis=1, keepdims=True)
            diff, spec = self._irradiance(X, n, return_spec_for=d[hit])
            rad[hit] = self._scale * (club.albedo * diff + 0.8 * spec)
        x0, y0, x1, y1 = box
        rad = rad.reshape(y1 - y0, x1 - x0)
        cov = hit.reshape(y1 - y0, x1 - x0).astype(np.float64)
        # shaft: a thin, moderately bright line
        heel, hands = club.shaft(t)
        pc = self.cam.to_cam(np.stack([heel, hands]))
        if np.all(pc[:, 2] > 0.1):
            uv = self.cam.project_cam(pc) - [x0, y0]
            m = np.zeros(rad.shape, np.uint8)
            wpx = max(2, int(self.cam.fx * 0.009 / pc[0, 2]))
            cv2.line(m, tuple(int(v) for v in uv[0]), tuple(int(v) for v in uv[1]), 1, wpx)
            sh = (m > 0) & (cov == 0)
            rad[sh] = 0.25 * self.s.ball_electrons
            cov[sh] = 1.0
        return rad, cov

    # ------------------------------------------------------------- exposure
    def expose_electrons(self, pulse_times: list[float], width_s: float, ball_state: BallState,
                         club: ClubMotion | None = None) -> np.ndarray:
        """Noise-free electrons for one exposure containing the given strobe pulses."""
        scale_w = width_s / (self.s.ref_pulse_us * 1e-6)
        W, H = self.cam.image_size
        e = self.mat * (len(pulse_times) * scale_w)
        for t in pulse_times:
            layers = []
            st = ball_state(t)
            if st is not None:
                st0 = ball_state(t - width_s / 2)
                st1 = ball_state(t + width_s / 2)
                move_px = np.linalg.norm(st1[0] - st0[0]) * self.cam.fx / max(0.2, self.cam.to_cam(st[0])[2])
                k = int(np.clip(math.ceil(move_px / 1.2), 1, self.s.max_time_samples))
                states = [ball_state(t + ((j + 0.5) / k - 0.5) * width_s) for j in range(k)]
                box = self._bbox(np.stack([s_[0] for s_ in states]), BALL_RADIUS_M)
                if box is not None:
                    layers.append((float(np.linalg.norm(st[0] - self.cam.center)), box,
                                   self._ball_layer(box, states)))
            if club is not None:
                c = club.center(t)
                heel, hands = club.shaft(t)
                box = self._bbox(np.stack([c, heel, hands]), max(club.semi_axes))
                if box is not None:
                    layers.append((float(np.linalg.norm(c - self.cam.center)), box, self._club_layer(box, club, t)))
            layers.sort(key=lambda L: -L[0])            # far first
            if len(layers) == 1:
                _, (x0, y0, x1, y1), (rad, cov) = layers[0]
                e[y0:y1, x0:x1] += (scale_w * cov * (rad - self.mat[y0:y1, x0:x1])).astype(np.float32)
            elif layers:
                ux0 = min(L[1][0] for L in layers); uy0 = min(L[1][1] for L in layers)
                ux1 = max(L[1][2] for L in layers); uy1 = max(L[1][3] for L in layers)
                base = self.mat[uy0:uy1, ux0:ux1].astype(np.float64)
                lay = base.copy()
                for _, (x0, y0, x1, y1), (rad, cov) in layers:
                    sl = (slice(y0 - uy0, y1 - uy0), slice(x0 - ux0, x1 - ux0))
                    lay[sl] = lay[sl] * (1 - cov) + rad * cov
                e[uy0:uy1, ux0:ux1] += (scale_w * (lay - base)).astype(np.float32)
        if self.s.psf_sigma_px > 0:
            e = cv2.GaussianBlur(e, (0, 0), self.s.psf_sigma_px)
        return e

    def to_raw(self, electrons: np.ndarray) -> np.ndarray:
        """Shot noise + read noise + gain + black level + saturation -> uint16 raw."""
        e = np.clip(electrons, 0, None)
        noise = self.rng.standard_normal(e.shape, dtype=np.float32)
        sig = np.sqrt(e + self.s.read_noise_e ** 2)
        dn = self.s.black_level + self.s.gain_dn_per_e * (e + sig * noise)
        return np.clip(np.rint(dn), 0, self.s.full_scale).astype(np.uint16)
