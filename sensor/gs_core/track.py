"""Stereo matching, strobe-pulse association and trajectory fit.

Inputs are the ball images found in each exposure of both cameras and the
exact strobe timing reported by the MCU. Steps:

1. ``stereo_match``: pair ball images between the cameras. Because the stereo
   baseline is parallel to the flight direction, successive ball images lie
   near one epipolar line; the ambiguity is resolved with the distance implied
   by each image's apparent size (a wrong pairing triangulates several cm too
   near or too far), plus the ray gap (epipolar error).
2. ``associate``: assign 3-D points to strobe pulses. Every pair of points with
   every plausible pair of pulse slots is a hypothesis (velocity follows from
   the known pulse times); the winner explains the most points, comes from the
   rest position, and has the fewest pulses that should have produced a
   visible image but did not. Clutter (club head) and wrong stereo pairings
   fall out as outliers. Images at the rest position are pulses fired before
   impact. Pulses whose image left the frame or merged into a cluster of
   overlapping images are simply not used.
3. ``fit_trajectory``: weighted least squares, constant velocity + gravity
   (or a rolling model for putts), with parameter covariance.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment

from .ballfit import BallImage
from .calibration import RigCalibration, midpoint_triangulate
from .geometry import BALL_DIAMETER_M, launch_from_velocity
from .physics import AeroModel

GRAVITY = np.array([0.0, 0.0, -9.81])
_DRAG = AeroModel()


@dataclass
class StereoBall:
    exposure: int
    p: np.ndarray             # world position of the ball centre
    gap_m: float              # distance between the two centre rays
    a: BallImage              # cam0 image
    b: BallImage              # cam1 image
    slot: int = -1            # index into the slot list after association
    at_rest: bool = False


@dataclass
class Slot:
    exposure: int
    pulse: int
    t: float                  # seconds, centre of the strobe pulse


@dataclass
class TrackConfig:
    max_gap_m: float = 0.002
    max_size_dist_err: float = 0.035      # |size-implied distance / triangulated - 1|
    rest_tol_m: float = 0.004
    assoc_tol_m: float = 0.003
    min_speed: float = 0.25
    max_speed: float = 110.0
    sigma_lateral_m: float = 0.00012
    sigma_depth_m: float = 0.00040
    max_hypotheses_scored: int = 300
    min_points: int = 3                   # two points fit any pulse pair: not a measurement


# -------------------------------------------------------------------- stereo
def stereo_match(imgs0: list[BallImage], imgs1: list[BallImage], rig: RigCalibration, exposure: int,
                 cfg: TrackConfig | None = None) -> list[StereoBall]:
    cfg = cfg or TrackConfig()
    if not imgs0 or not imgs1:
        return []
    c0, c1 = rig.pair
    n0, n1 = len(imgs0), len(imgs1)
    cost = np.full((n0, n1), 1e6)
    pts = np.zeros((n0, n1, 3))
    gaps = np.zeros((n0, n1))
    for i, a in enumerate(imgs0):
        for j, b in enumerate(imgs1):
            X, gap = midpoint_triangulate(c0.center, a.ray_w, c1.center, b.ray_w)
            if (X - c0.center) @ a.ray_w <= 0:
                continue
            e0 = 0.0 if a.interpolated else a.dist_m / np.linalg.norm(X - c0.center) - 1
            e1 = 0.0 if b.interpolated else b.dist_m / np.linalg.norm(X - c1.center) - 1
            if gap > cfg.max_gap_m or abs(e0) > cfg.max_size_dist_err or abs(e1) > cfg.max_size_dist_err:
                continue
            cost[i, j] = (gap / 0.0005) ** 2 + (e0 / 0.012) ** 2 + (e1 / 0.012) ** 2
            pts[i, j], gaps[i, j] = X, gap
    ri, ci = linear_sum_assignment(cost)
    return [StereoBall(exposure, pts[i, j], float(gaps[i, j]), imgs0[i], imgs1[j])
            for i, j in zip(ri, ci) if cost[i, j] < 1e5]


# ---------------------------------------------------------------- trajectory
@dataclass
class Trajectory:
    t_ref: float
    p0: np.ndarray
    v: np.ndarray                 # velocity at t_ref
    accel: np.ndarray             # constant acceleration used by the model
    model: str                    # "ballistic" | "rolling"
    cov: np.ndarray               # 6x6 covariance of (p0, v)
    rms_m: float
    n: int
    t_impact: float = float("nan")

    def position(self, t: np.ndarray | float) -> np.ndarray:
        tau = np.asarray(t, float) - self.t_ref
        return self.p0 + self.v * tau[..., None] + 0.5 * self.accel * (tau ** 2)[..., None]

    def velocity(self, t: float) -> np.ndarray:
        return self.v + self.accel * (t - self.t_ref)


def _point_cov(p: np.ndarray, rig: RigCalibration, cfg: TrackConfig) -> np.ndarray:
    n = p - 0.5 * (rig.cam0.center + rig.cam1.center)
    n /= np.linalg.norm(n)
    return cfg.sigma_lateral_m ** 2 * np.eye(3) + (cfg.sigma_depth_m ** 2 - cfg.sigma_lateral_m ** 2) * np.outer(n, n)


def point_quality(b: "StereoBall") -> float:
    """Relative precision of a triangulated centre: an image with only part of its
    edge visible (overlapping neighbours, frame border) is located less well,
    mostly along the hidden direction; interpolated chain images least."""
    q = []
    for im in (b.a, b.b):
        q.append(0.25 if im.interpolated else float(np.clip(im.arc_frac / 0.8, 0.2, 1.0)))
    return min(q)


def fit_trajectory(pts: np.ndarray, t: np.ndarray, rig: RigCalibration, cfg: TrackConfig | None = None,
                   model: str = "ballistic", quality: np.ndarray | None = None) -> Trajectory:
    """Weighted LSQ of p(t) = p0 + v tau + a tau^2 / 2. ``rolling``: a = k * v_hat
    (k fitted when >= 4 points span > 20 ms, else 0), no gravity."""
    cfg = cfg or TrackConfig()
    pts = np.asarray(pts, float)
    t = np.asarray(t, float)
    n = len(t)
    t_ref = float(t.mean())
    tau = t - t_ref
    qual = np.ones(n) if quality is None else np.clip(np.asarray(quality, float), 0.05, 1.0)
    Ws = [q * np.linalg.cholesky(np.linalg.inv(_point_cov(p, rig, cfg))).T for p, q in zip(pts, qual)]
    accel = GRAVITY.copy() if model == "ballistic" else np.zeros(3)
    fit_k = model == "rolling" and n >= 4 and (t.max() - t.min()) > 0.020
    uhat = np.zeros(3)
    wt = np.ones(n)
    # ballistic: a second pass adds aerodynamic drag at the first-pass velocity
    # (~25 m/s^2 at 75 m/s; ignoring it biases the impact speed by ~0.1 m/s)
    n_pass = (2 if fit_k else 1) + (2 if n >= 4 else 0) + (1 if model == "ballistic" else 0)
    for it in range(n_pass):
        use_k = fit_k and it >= 1
        rows, rhs = [], []
        for i in range(n):
            A = np.zeros((3, 7 if use_k else 6))
            A[:, :3] = np.eye(3)
            A[:, 3:6] = np.eye(3) * tau[i]
            y = pts[i] - (0 if use_k else 0.5 * accel * tau[i] ** 2)
            if use_k:
                A[:, 6] = 0.5 * uhat * tau[i] ** 2
            rows.append(wt[i] * (Ws[i] @ A))
            rhs.append(wt[i] * (Ws[i] @ y))
        A = np.concatenate(rows)
        y = np.concatenate(rhs)
        sol, *_ = np.linalg.lstsq(A, y, rcond=None)
        if fit_k and it == 0:
            uhat = sol[3:6] / np.linalg.norm(sol[3:6])
        elif use_k:
            accel = sol[6] * uhat
        if model == "ballistic" and it == 0:
            accel = _DRAG.accel(sol[3:6], None)
        if n >= 4:
            # robust re-weighting (Cauchy): one image disturbed by the club must not tilt the fit
            mp = sol[:3] + sol[3:6] * tau[:, None] + 0.5 * accel * (tau ** 2)[:, None]
            r = np.linalg.norm(mp - pts, axis=1)
            sc = max(1.5e-4, 1.4826 * float(np.median(r)))
            wt = 1.0 / np.sqrt(1.0 + (r / (2.0 * sc)) ** 2)
    res = (A @ sol - y)
    dof = max(1, 3 * n - A.shape[1])
    chi2 = float(res @ res) / dof if 3 * n > A.shape[1] else 1.0
    cov = np.linalg.inv(A.T @ A)[:6, :6] * max(1.0, chi2)
    p0, v = sol[:3], sol[3:6]
    model_pts = p0 + v * tau[:, None] + 0.5 * accel * (tau ** 2)[:, None]
    rms = float(np.sqrt(np.mean(np.sum((model_pts - pts) ** 2, 1))))
    return Trajectory(t_ref, p0, v, accel, model, cov, rms, n)


def impact_time(tr: Trajectory, rest: np.ndarray | None, fallback: float) -> float:
    """Time at which the fitted path is closest to the rest position."""
    if rest is None:
        return fallback
    t = tr.t_ref
    for _ in range(4):
        d = tr.position(t) - rest
        vel = tr.velocity(t)
        t -= float(d @ vel) / max(1e-9, float(vel @ vel))
    return t


@dataclass
class LaunchEstimate:
    speed_mps: float
    vla_deg: float
    hla_deg: float
    sigma_speed: float
    sigma_vla: float
    sigma_hla: float
    velocity: np.ndarray
    trajectory: Trajectory
    balls: list[StereoBall] = field(default_factory=list)     # inliers, slot filled
    n_rest: int = 0
    n_clutter: int = 0


def launch_from_trajectory(tr: Trajectory) -> tuple[float, float, float, float, float, float]:
    """(speed, vla, hla, sigma...) at the impact time, with first-order error propagation."""
    dt = tr.t_impact - tr.t_ref
    if tr.model == "ballistic":
        dt = float(np.clip(dt, -0.05, 0.05))
    v = tr.v + tr.accel * dt
    sp, vla, hla = launch_from_velocity(v)
    J = np.zeros((3, 3))
    for k in range(3):
        dv = np.zeros(3)
        dv[k] = 1e-4
        a, b, c = launch_from_velocity(v + dv)
        J[:, k] = [(a - sp) / 1e-4, (b - vla) / 1e-4, (c - hla) / 1e-4]
    C = J @ tr.cov[3:6, 3:6] @ J.T
    s = np.sqrt(np.clip(np.diag(C), 0, None))
    return sp, vla, hla, float(s[0]), float(s[1]), float(s[2])


# --------------------------------------------------------------- association
def _visible(rig: RigCalibration, P: np.ndarray, margin_px: float) -> np.ndarray:
    ok = np.ones(P.shape[:-1], bool)
    for cam in rig.pair:
        pc = cam.to_cam(P)
        uv = cam.project_cam(np.where(pc[..., 2:3] > 0.05, pc, [0, 0, 1.0]))
        W, H = cam.image_size
        ok &= (pc[..., 2] > 0.05) & (uv[..., 0] > margin_px) & (uv[..., 0] < W - margin_px) \
            & (uv[..., 1] > margin_px) & (uv[..., 1] < H - margin_px)
    return ok


def associate(balls: list[StereoBall], slots: list[Slot], rig: RigCalibration,
              rest: np.ndarray | None, cfg: TrackConfig | None = None) -> LaunchEstimate | None:
    """Assign triangulated ball images to strobe pulses and fit the launch."""
    cfg = cfg or TrackConfig()
    for b in balls:
        b.at_rest = rest is not None and float(np.linalg.norm(b.p - rest)) < cfg.rest_tol_m
        b.slot = -1
    mov = [b for b in balls if not b.at_rest]
    if len(mov) < 2 or len(slots) < 2:
        return None
    P = np.stack([b.p for b in mov])
    pe = np.array([b.exposure for b in mov])
    st = np.array([s.t for s in slots])
    se = np.array([s.exposure for s in slots])
    ghost_exp = {b.exposure for b in balls if b.at_rest}
    M = len(mov)
    r_px = float(np.mean([b.a.r_px for b in mov]))

    hyp_p, hyp_v, hyp_t, hyp_g = [], [], [], []
    for ia in range(M):
        for ib in range(M):
            if ia == ib:
                continue
            sa = np.nonzero(se == pe[ia])[0]
            sb = np.nonzero(se == pe[ib])[0]
            A, B = np.meshgrid(sa, sb, indexing="ij")
            A, B = A.ravel(), B.ravel()
            dt = st[B] - st[A]
            k = dt > 1e-6
            if not np.any(k):
                continue
            A, dt = A[k], dt[k]
            for g in (GRAVITY, np.zeros(3)):
                v = (P[ib] - P[ia])[None, :] / dt[:, None] - 0.5 * g[None, :] * dt[:, None]
                sp = np.linalg.norm(v, axis=1)
                hor = np.hypot(v[:, 0], v[:, 1])
                ok = (sp > cfg.min_speed) & (sp < cfg.max_speed) & (v[:, 0] > 0.6 * hor) \
                    & (v[:, 2] > -0.2 * sp) & (v[:, 2] < 0.95 * sp)
                if not np.any(ok):
                    continue
                hyp_p.append(np.repeat(P[ia][None, :], ok.sum(), 0))
                hyp_v.append(v[ok])
                hyp_t.append(st[A[ok]])
                hyp_g.append(np.repeat(g[None, :], ok.sum(), 0))
    if not hyp_p:
        return None
    hp, hv, ht, hg = (np.concatenate(x) for x in (hyp_p, hyp_v, hyp_t, hyp_g))
    tau = st[None, :] - ht[:, None]                                   # (H, S)
    pred = hp[:, None, :] + hv[:, None, :] * tau[..., None] + 0.5 * hg[:, None, :] * (tau ** 2)[..., None]
    dist = np.linalg.norm(pred[:, :, None, :] - P[None, None, :, :], axis=-1)       # (H, S, M)
    dist = np.where(se[None, :, None] == pe[None, None, :], dist, 1e9)
    hit = dist < cfg.assoc_tol_m
    n_in = hit.any(axis=1).sum(axis=1)                                 # distinct points explained
    order = np.argsort(-n_in, kind="stable")[: cfg.max_hypotheses_scored]
    best, best_score = -1, -1e9
    for h in order:
        if n_in[h] < cfg.min_points:
            break
        sp = float(np.linalg.norm(hv[h]))
        score = float(n_in[h])
        t_star = ht[h]
        if rest is not None:
            t_star = ht[h] + float((rest - hp[h]) @ hv[h]) / (sp * sp)
            tt = t_star - ht[h]
            d_rest = float(np.linalg.norm(hp[h] + hv[h] * tt + 0.5 * hg[h] * tt * tt - rest))
            score -= min(3.0, d_rest / 0.012)
        slot_hit = hit[h].any(axis=1)
        vis = _visible(rig, pred[h], 0.9 * r_px) & (st > t_star + 2e-4)
        for e in np.unique(se):
            k = np.nonzero(se == e)[0]
            if len(k) > 1 and sp * float(np.min(np.diff(st[k]))) < 0.6 * BALL_DIAMETER_M:
                vis[k] = False                      # overlapping cluster: misses are expected
            if e in ghost_exp and not np.any(st[k] < t_star):
                score -= 1.0                        # an image at rest needs a pulse before impact
        score -= 0.6 * float(np.sum(vis & ~slot_hit))
        score -= 0.3 * float(M - n_in[h])
        score -= 0.05 * float(np.sum(np.where(hit[h], dist[h], 0)) / max(1, hit[h].sum()) / cfg.assoc_tol_m)
        if score > best_score:
            best, best_score = h, score
    if best < 0:
        return None

    # refine: fit, re-associate with the fitted model, fit again
    p_h, v_h, t_h, g_h = hp[best], hv[best], ht[best], hg[best]
    tr = None
    used: dict[int, int] = {}
    for it in range(3):
        if tr is None:
            tau = st - t_h
            pr = p_h + v_h * tau[:, None] + 0.5 * g_h * (tau ** 2)[:, None]
        else:
            pr = tr.position(st)
        d = np.linalg.norm(pr[:, None, :] - P[None, :, :], axis=-1)
        d = np.where(se[:, None] == pe[None, :], d, 1e9)
        used = {}
        for m in range(M):
            s = int(np.argmin(d[:, m]))
            if d[s, m] < cfg.assoc_tol_m and (s not in used or d[s, m] < d[s, used[s]]):
                used[s] = m
        if len(used) < cfg.min_points:
            return None
        ss = sorted(used)
        pts = P[[used[s] for s in ss]]
        qq = np.array([point_quality(mov[used[s]]) for s in ss])
        tt = st[ss]
        tb = fit_trajectory(pts, tt, rig, cfg, "ballistic", qq)
        tr = tb
        if rest is not None and np.all(np.abs(pts[:, 2] - rest[2]) < 0.004):
            trr = fit_trajectory(pts, tt, rig, cfg, "rolling", qq)
            if abs(trr.v[2]) < 0.05 * np.linalg.norm(trr.v) + 0.05 and trr.rms_m <= 1.5 * tb.rms_m + 1e-4:
                tr = trr
    tr.t_impact = impact_time(tr, rest, float(st.min()))
    sp, vla, hla, s_sp, s_vla, s_hla = launch_from_trajectory(tr)
    out = []
    for s, m in sorted(used.items()):
        mov[m].slot = s
        out.append(mov[m])
    dt = tr.t_impact - tr.t_ref if tr.model == "rolling" else float(np.clip(tr.t_impact - tr.t_ref, -0.05, 0.05))
    return LaunchEstimate(sp, vla, hla, s_sp, s_vla, s_hla, tr.v + tr.accel * dt, tr, out,
                          n_rest=len(balls) - len(mov), n_clutter=M - len(used))
