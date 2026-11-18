"""Short-range golf-ball flight model (gravity, drag, Magnus).

Used by the synthetic data generator (true flight with drag and lift). The
measurement fit uses constant velocity + gravity over the few ms observed;
over 4 ms at 75 m/s drag changes the speed by ~0.1 m/s, which the fit
averages (reported velocity is at the impact time, extrapolated ~2 ms).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import BALL_RADIUS_M

G = np.array([0.0, 0.0, -9.81])
BALL_MASS_KG = 0.04593


@dataclass
class AeroModel:
    rho: float = 1.204          # air density kg/m^3 (sea level, 20 C)
    cd: float = 0.24            # drag coefficient (typical for Re ~ 1e5..2e5)
    cd_spin: float = 0.15       # drag increase per unit spin factor
    cl_a: float = 1.99          # Cl = cl_a*S + cl_b*S^2 (S = spin factor r*w/v), clipped
    cl_b: float = -3.25
    cl_max: float = 0.32

    def accel(self, v: np.ndarray, omega: np.ndarray | None) -> np.ndarray:
        """Acceleration (m/s^2) for velocity ``v`` and spin ``omega`` (rad/s)."""
        sp = float(np.linalg.norm(v))
        if sp < 1e-6:
            return G.copy()
        area = math.pi * BALL_RADIUS_M ** 2
        q = 0.5 * self.rho * sp * sp * area / BALL_MASS_KG
        a = G.copy()
        w = 0.0 if omega is None else float(np.linalg.norm(omega))
        s = BALL_RADIUS_M * w / sp
        cd = self.cd + self.cd_spin * s
        a -= q * cd * v / sp
        if w > 1e-6:
            cl = min(self.cl_max, max(0.0, self.cl_a * s + self.cl_b * s * s))
            lift_dir = np.cross(omega, v)
            n = np.linalg.norm(lift_dir)
            if n > 1e-9:
                a += q * cl * lift_dir / n
        return a


def integrate(p0: np.ndarray, v0: np.ndarray, omega: np.ndarray | None, times: np.ndarray,
              aero: AeroModel | None = None, dt: float = 2.5e-4) -> tuple[np.ndarray, np.ndarray]:
    """RK4 integration from state (p0, v0) at t=0 to each of ``times`` (may be negative).

    Returns (positions (N,3), velocities (N,3)).
    """
    aero = aero or AeroModel()
    times = np.asarray(times, float)
    order = np.argsort(times)
    out_p = np.zeros((times.size, 3))
    out_v = np.zeros((times.size, 3))
    # integrate forward for t >= 0 and backward for t < 0
    for sign in (1.0, -1.0):
        idx = [i for i in order if (times[i] >= 0) == (sign > 0)]
        if sign < 0:
            idx = idx[::-1]
        p = np.array(p0, float)
        v = np.array(v0, float)
        t = 0.0
        for i in idx:
            target = times[i]
            while abs(target - t) > 1e-12:
                h = sign * min(dt, abs(target - t))
                k1v = aero.accel(v, omega)
                k1p = v
                k2v = aero.accel(v + 0.5 * h * k1v, omega)
                k2p = v + 0.5 * h * k1v
                k3v = aero.accel(v + 0.5 * h * k2v, omega)
                k3p = v + 0.5 * h * k2v
                k4v = aero.accel(v + h * k3v, omega)
                k4p = v + h * k3v
                p = p + h / 6.0 * (k1p + 2 * k2p + 2 * k3p + k4p)
                v = v + h / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
                t += h
            out_p[i] = p
            out_v[i] = v
    return out_p, out_v
