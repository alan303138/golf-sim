"""Markerless spin: 3-D rotation of the ball between strobe images.

Each ball image is unwrapped onto a chart of the sphere (azimuthal equidistant
projection centred on the direction ball -> camera), which removes the
perspective/foreshortening of the changing viewing direction. Two feature
channels are computed on the chart:

* shading channel - dimples are visible only as shading: the normalised image
  ``q = I / smooth(I) - 1`` equals, to first order, the height gradient along
  the tangential light direction ``u`` (scaled by a known factor). ``q`` is
  therefore NOT a body-fixed texture: when the ball turns, the light direction
  relative to the surface changes. We rescale ``q`` by the known photometric
  factor and differentiate once more along ``u``; the result approximates the
  second directional derivative of the smoothed height map, whose response at
  a dimple centre has the same sign for every light direction (correlation
  between two light directions >= 1/3 instead of cos(angle), which goes
  negative).
* albedo channel - prints (logo, number, a drawn line or dots) are body-fixed
  and light-independent; a wider low-pass of ``q`` keeps them and averages the
  dimple dipoles out.

The angular velocity ``omega`` (constant during the few ms observed) is found
by maximising the mean normalised correlation over image pairs of the same
camera: a coarse grid over (backspin, sidespin) using consecutive images, then
a local optimisation over all three components using all pairs. Because every
pair is predicted from ONE omega and the known pulse times, three or more
images also break most of the aliasing caused by the symmetry of dimple
patterns.

The confidence combines the height of the peak above the noise floor of the
score landscape, its margin over the best competing peak, the absolute
correlation and whether each camera alone prefers the same solution. A plain
(texture-less or out-of-focus) ball yields no significant peak, hence a low
confidence, and the pipeline then falls back to an empirical estimate.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import cv2
import numpy as np
from scipy.optimize import minimize

from .geometry import BALL_RADIUS_M, RPM_TO_RADS, CameraModel, rotation_matrix, spin_frame, spin_from_vector


@dataclass
class SpinConfig:
    chart_n: int = 121
    chart_max_deg: float = 60.0
    flat_sigma: float = 8.0           # chart px, illumination flat-fielding
    shade_sigma: float = 2.0          # chart px, smoothing before the directional derivative
    albedo_sigma: float = 4.0         # chart px
    albedo_ref: float = 0.035         # contrast that counts as "one sigma" of albedo signal
    min_ltan: float = 0.10            # |tangential light| below which shading carries no signal
    coarse_samples: int = 300
    fine_samples: int = 500
    max_pair_span: float = 3.5        # use image pairs up to this many shortest intervals apart
    n_refine: int = 2
    max_grid: int = 900
    maxfev: int = 60
    step_back_rad: float = 0.035      # coarse grid step, rotation per shortest pulse interval
    step_side_rad: float = 0.05
    max_side_ratio: float = 0.7
    min_overlap: float = 0.15
    marked: bool = False              # marked-ball mode: albedo channel only
    glint_mask_deg: float = 14.0      # mask around the predicted specular highlight of each LED
    peak_sep_rad: float = 0.25        # rotation difference (per shortest interval) of distinct solutions
    measured_threshold: float = 0.5


@dataclass
class BallChart:
    cam: str
    t: float
    basis: np.ndarray                 # rows e1, e2, c (world); c = ball -> camera
    feat: np.ndarray                  # (N, N, C) float32, zero where invalid
    valid: np.ndarray                 # (N, N) float32 0/1
    scale: float                      # chart px per radian
    quality: float                    # fraction of the chart that is usable


@dataclass
class SpinEstimate:
    omega: np.ndarray                 # rad/s, world frame
    total_rpm: float
    axis_deg: float
    confidence: float
    measured: bool
    sigma_rpm: float = float("nan")
    sigma_axis_deg: float = float("nan")
    details: dict = field(default_factory=dict)


# ------------------------------------------------------------------- charts
def _nconv(img: np.ndarray, w: np.ndarray, sigma: float) -> np.ndarray:
    num = cv2.GaussianBlur(img * w, (0, 0), sigma)
    den = cv2.GaussianBlur(w, (0, 0), sigma)
    return num / np.maximum(den, 1e-3)


def make_chart(frame_fg: np.ndarray, cam: CameraModel, center_w: np.ndarray, t: float,
               lights_w: np.ndarray, sat_level: float, exclude: list[tuple[float, float, float]] | None = None,
               cfg: SpinConfig | None = None) -> BallChart:
    """Unwrap one ball image. ``frame_fg``: background-subtracted frame (float32);
    ``exclude``: (u, v, r) discs of other ball images/clutter to mask out."""
    cfg = cfg or SpinConfig()
    N = cfg.chart_n
    pmax = math.radians(cfg.chart_max_deg)
    c = cam.center - center_w
    dist = float(np.linalg.norm(c))
    c /= dist
    e1 = np.cross([0.0, 0.0, 1.0], c)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(c, e1)
    g = np.linspace(-pmax, pmax, N)
    A, B = np.meshgrid(g, g)                       # A along e1 (columns), B along e2 (rows)
    psi = np.hypot(A, B)
    sinc = np.where(psi > 1e-9, np.sin(psi) / np.maximum(psi, 1e-9), 1.0)
    n = (sinc * A)[..., None] * e1 + (sinc * B)[..., None] * e2 + np.cos(psi)[..., None] * c
    X = center_w + BALL_RADIUS_M * n
    uv = cam.project(X).astype(np.float32)
    H, W = frame_fg.shape
    img = cv2.remap(frame_fg, uv[..., 0], uv[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                    borderValue=-1e6)
    valid = (psi <= pmax) & (img > -1e5) & (img < sat_level)
    if exclude:
        for (u, v, r) in exclude:
            valid &= np.hypot(uv[..., 0] - u, uv[..., 1] - v) > r * 1.04
    # light: sum of unit directions to the LEDs weighted by 1/d^2 (LEDs below the local horizon dropped)
    # Lambertian model of the smooth sphere: M = sum_k E_k max(n.l_k, 0); Lw = sum_k E_k l_k (lit LEDs)
    L = np.zeros_like(n)
    M = np.zeros(n.shape[:-1])
    for lp in lights_w:
        lv = lp - X
        d2 = np.sum(lv * lv, -1, keepdims=True)
        l = lv / np.sqrt(d2)
        ndl_k = np.sum(l * n, -1, keepdims=True)
        L += l / d2 * (ndl_k > 0.0)
        M += (np.clip(ndl_k, 0, None) / d2)[..., 0]
        hv = l + c                                   # half vector (view direction ~ c)
        hv /= np.linalg.norm(hv, axis=-1, keepdims=True)
        valid &= np.sum(hv * n, -1) < math.cos(math.radians(cfg.glint_mask_deg))
    if len(lights_w) == 0:
        L = np.broadcast_to(c, n.shape).copy()
        M = np.clip(np.sum(n * c, -1), 0, None)
    Mn = M / max(1e-12, float(M.max()))
    L /= np.maximum(np.linalg.norm(L, axis=-1, keepdims=True), 1e-12)
    ndl = np.sum(n * L, -1)
    ltan = L - ndl[..., None] * n
    lt = np.linalg.norm(ltan, axis=-1)
    valid &= Mn > 0.3
    w = valid.astype(np.float32)
    # flat field: divide by the model shading, then by a smooth residual gain
    ratio = np.where(valid, img / np.maximum(Mn, 0.05), 0).astype(np.float32)
    smooth = _nconv(ratio, w, cfg.flat_sigma)
    q = np.where(valid, ratio / np.maximum(smooth, 1.0) - 1.0, 0).astype(np.float32)
    # specular glints and other photometric outliers
    hl = cv2.dilate((q > 0.6).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    valid &= ~hl
    w = valid.astype(np.float32)
    q = np.where(valid, np.clip(q, -1, 0.6), 0).astype(np.float32)
    # trim the rim of the valid region (filters would mix in invalid pixels)
    w_in = (cv2.erode(w, np.ones((5, 5), np.uint8)) > 0)
    chans = []
    # --- albedo channel
    alb = _nconv(q, w, cfg.albedo_sigma) / cfg.albedo_ref
    alb = np.where(w_in, alb, 0)
    chans.append(alb)
    if not cfg.marked:
        # --- shading channel: D_u (G * D_u h),  D_u h ~ -q * (n.l) / |l_tan|
        ok = w_in & (lt > cfg.min_ltan)
        gsh = np.where(valid & (lt > cfg.min_ltan), -q * ndl / np.maximum(lt, cfg.min_ltan), 0).astype(np.float32)
        gs = _nconv(gsh, (valid & (lt > cfg.min_ltan)).astype(np.float32), cfg.shade_sigma)
        d_b, d_a = np.gradient(gs)
        # chart-plane direction of the tangential light: u = (l_tan . dn/da, l_tan . dn/db), normalised
        t_a = np.gradient(n, axis=1)
        t_b = np.gradient(n, axis=0)
        ua = np.sum(ltan * t_a, -1) / np.maximum(np.sum(t_a * t_a, -1), 1e-12)
        ub = np.sum(ltan * t_b, -1) / np.maximum(np.sum(t_b * t_b, -1), 1e-12)
        un = np.maximum(np.hypot(ua, ub), 1e-12)
        m = (d_a * ua + d_b * ub) / un
        ok &= cv2.erode(ok.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
        if ok.sum() > 50:
            mm = m[ok]
            m = (m - mm.mean()) / (mm.std() + 1e-9)
        m = np.where(ok, np.clip(m, -4, 4), 0)
        chans.insert(0, m)
    feat = np.stack(chans, -1).astype(np.float32)
    use = w_in.astype(np.float32)
    return BallChart(cam.name, t, np.stack([e1, e2, c]), feat * use[..., None], use, (N - 1) / (2 * pmax),
                     float(use.sum() / max(1.0, float((psi <= pmax).sum()))))


def _chart_normals(ch: BallChart, n_samples: int, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """World-frame surface normals and features of up to ``n_samples`` valid chart pixels."""
    ys, xs = np.nonzero(ch.valid > 0)
    if len(xs) > n_samples:
        k = rng.choice(len(xs), n_samples, replace=False)
        ys, xs = ys[k], xs[k]
    N = ch.valid.shape[0]
    a = (xs - (N - 1) / 2) / ch.scale
    b = (ys - (N - 1) / 2) / ch.scale
    psi = np.hypot(a, b)
    sinc = np.where(psi > 1e-9, np.sin(psi) / np.maximum(psi, 1e-9), 1.0)
    n = (sinc * a)[:, None] * ch.basis[0] + (sinc * b)[:, None] * ch.basis[1] + np.cos(psi)[:, None] * ch.basis[2]
    return n, ch.feat[ys, xs]


def _sample(ch: BallChart, n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Bilinear lookup of chart features at world normals ``n`` (..., 3). Returns (feat (..., C), weight (...))."""
    cc = n @ ch.basis.T
    c3 = np.clip(cc[..., 2], -1, 1)
    psi = np.arccos(c3)
    s = psi / np.maximum(np.sqrt(1 - c3 * c3), 1e-9) * ch.scale
    N = ch.valid.shape[0]
    x = cc[..., 0] * s + (N - 1) / 2
    y = cc[..., 1] * s + (N - 1) / 2
    inside = (x >= 0) & (x < N - 1) & (y >= 0) & (y < N - 1) & (c3 > 0.3)
    x0 = np.clip(np.floor(x), 0, N - 2).astype(np.intp)
    y0 = np.clip(np.floor(y), 0, N - 2).astype(np.intp)
    fx = (x - x0)[..., None]
    fy = (y - y0)[..., None]
    F, V = ch.feat, ch.valid
    f = (F[y0, x0] * (1 - fx) * (1 - fy) + F[y0, x0 + 1] * fx * (1 - fy)
         + F[y0 + 1, x0] * (1 - fx) * fy + F[y0 + 1, x0 + 1] * fx * fy)
    v = np.minimum(np.minimum(V[y0, x0], V[y0, x0 + 1]), np.minimum(V[y0 + 1, x0], V[y0 + 1, x0 + 1]))
    return f, v * inside


# ------------------------------------------------------------------ scoring
class _Pair:
    def __init__(self, a: BallChart, b: BallChart, n_samples: int, rng: np.random.Generator) -> None:
        self.dt = b.t - a.t
        self.b = b
        self.R_view = b.basis.T @ a.basis           # maps chart a onto chart b pixel-for-pixel
        self.cam = a.cam
        self.n, self.f = _chart_normals(a, n_samples, rng)
        self.k = len(self.n)

    def score(self, omegas: np.ndarray, min_overlap: float, R: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        """NCC and overlap weight for each omega (H,3) (or for explicit rotations R (H,3,3))."""
        if R is None:
            R = rotation_matrix(omegas * self.dt)                       # (H,3,3)
        nr = np.einsum("hij,kj->hki", R, self.n)
        fb, w = _sample(self.b, nr)
        fa = self.f[None, :, :]
        num = np.sum(fa * fb * w[..., None], axis=(1, 2))
        da = np.sum(fa * fa * w[..., None], axis=(1, 2))
        db = np.sum(fb * fb * w[..., None], axis=(1, 2))
        cnt = w.sum(axis=1)
        ncc = num / np.sqrt(np.maximum(da * db, 1e-9))
        ok = cnt >= min_overlap * max(1, self.k)
        return np.where(ok, ncc, 0.0), np.where(ok, cnt, min_overlap * self.k)


_POOL = None


def _pool():
    global _POOL
    if _POOL is None:
        import os
        from concurrent.futures import ThreadPoolExecutor
        _POOL = ThreadPoolExecutor(max_workers=min(4, os.cpu_count() or 1), thread_name_prefix="spin")
    return _POOL


def _score_pairs(pairs: list[_Pair], omegas: np.ndarray, min_overlap: float, chunk: int = 256):
    """Overlap-weighted mean NCC per omega; also per-camera means."""
    H = len(omegas)
    tot = np.zeros(H)
    wsum = np.zeros(H)
    cams = sorted({p.cam for p in pairs})
    per = {c: [np.zeros(H), np.zeros(H)] for c in cams}
    jobs = [(p, s) for p in pairs for s in range(0, H, chunk)]
    if H >= 64 and len(jobs) > 1:
        # coarse grid: spread over the cores (numpy releases the GIL in the heavy parts)
        results = list(_pool().map(lambda j: j[0].score(omegas[j[1]:j[1] + chunk], min_overlap), jobs))
    else:
        results = [p.score(omegas[s:s + chunk], min_overlap) for p, s in jobs]
    for (p, s), (ncc, w) in zip(jobs, results):
        tot[s:s + chunk] += ncc * w
        wsum[s:s + chunk] += w
        per[p.cam][0][s:s + chunk] += ncc * w
        per[p.cam][1][s:s + chunk] += w
    return tot / np.maximum(wsum, 1e-9), {c: v[0] / np.maximum(v[1], 1e-9) for c, v in per.items()}


def estimate_spin(charts: list[BallChart], v_dir: np.ndarray, max_rpm: float = 13000.0,
                  cfg: SpinConfig | None = None, seed: int = 0) -> SpinEstimate | None:
    """Angular velocity from >= 2 charts (any mix of cameras and pulse times)."""
    cfg = cfg or SpinConfig()
    rng = np.random.default_rng(seed)
    by_cam: dict[str, list[BallChart]] = {}
    for ch in charts:
        if ch.quality > 0.25:
            by_cam.setdefault(ch.cam, []).append(ch)
    adj: list[_Pair] = []
    allp: list[_Pair] = []
    for cam, lst in by_cam.items():
        lst.sort(key=lambda c: c.t)
        for i in range(len(lst) - 1):
            adj.append(_Pair(lst[i], lst[i + 1], cfg.coarse_samples, rng))
    if not adj:
        return None
    dt_min = min(p.dt for p in adj)
    for cam, lst in by_cam.items():
        for i in range(len(lst)):
            for j in range(i + 1, len(lst)):
                if lst[j].t - lst[i].t <= cfg.max_pair_span * dt_min + 1e-9:
                    allp.append(_Pair(lst[i], lst[j], cfg.fine_samples, rng))
    f, left, up = spin_frame(v_dir)
    w_max = max_rpm * RPM_TO_RADS
    # coarse grid in (backspin, sidespin); rotation steps refer to the shortest interval
    sb = cfg.step_back_rad / dt_min
    ss = cfg.step_side_rad / dt_min
    back = np.arange(-2 * sb, w_max + sb, sb)
    grid = []
    for wb in back:
        lim = cfg.max_side_ratio * max(abs(wb), 0.25 * w_max)
        ns = int(lim / ss)
        for k in range(-ns, ns + 1):
            grid.append((wb, k * ss))
    grid = np.array(grid)
    if len(grid) > cfg.max_grid:
        # large rotation per interval (slow ball, long interval): coarser grid, bounded cost
        k = math.sqrt(len(grid) / cfg.max_grid)
        return estimate_spin(charts, v_dir, max_rpm, replace(cfg, step_back_rad=cfg.step_back_rad * k,
                                                                 step_side_rad=cfg.step_side_rad * k,
                                                                 max_grid=10 ** 9), seed)
    omegas = -grid[:, 0:1] * left - grid[:, 1:2] * up
    sc, per_cam = _score_pairs(adj, omegas, cfg.min_overlap)
    med = float(np.median(sc))
    sig = float(1.4826 * np.median(np.abs(sc - med))) + 1e-6
    order = np.argsort(-sc)
    # distinct local peaks
    peaks: list[int] = []
    for h in order[:400]:
        if all(np.linalg.norm((omegas[h] - omegas[p]) * dt_min) > cfg.peak_sep_rad for p in peaks):
            peaks.append(int(h))
        if len(peaks) >= 4:
            break
    cands = []
    for h in peaks[: cfg.n_refine]:
        x0 = omegas[h]

        def cost(x: np.ndarray) -> float:
            s, _ = _score_pairs(allp, x[None, :], cfg.min_overlap)
            return -float(s[0])

        step = 0.02 / dt_min
        r = minimize(cost, x0, method="Nelder-Mead",
                     options={"initial_simplex": x0 + np.vstack([np.zeros(3), np.eye(3) * step]),
                              "xatol": 0.002 / dt_min, "fatol": 1e-4, "maxfev": cfg.maxfev})
        cands.append((-r.fun, r.x, float(sc[h])))
    cands.sort(key=lambda c: -c[0])
    s1, omega, s1_coarse = cands[0]
    # best competitor that did NOT converge onto the winner; when all refined
    # candidates merged, the best coarse point outside the winner's peak is scored
    others = [c for c in cands[1:] if np.linalg.norm((c[1] - omega) * dt_min) > cfg.peak_sep_rad]
    far = np.linalg.norm((omegas - omega) * dt_min, axis=1) > cfg.peak_sep_rad
    if others:
        s2 = others[0][0]
    elif far.any():
        hf = int(np.argmax(np.where(far, sc, -9)))
        s2 = float(_score_pairs(allp, omegas[hf:hf + 1], cfg.min_overlap)[0][0])
    else:
        s2 = 0.0
    bg = sc[far] if far.sum() > 20 else sc
    med = float(np.median(bg))
    sig = float(1.4826 * np.median(np.abs(bg - med))) + 1e-3
    z = (float(sc[peaks[0]]) - med) / sig
    margin_c = 1.0 - max(0.0, float(bg.max()) - med) / max(1e-6, float(sc[peaks[0]]) - med)
    margin_f = 1.0 - max(0.0, s2 - med) / max(1e-6, s1 - med)
    # null hypothesis: features fixed to the camera/light (glints, residual shading,
    # sensor pattern) - they align when chart b is mapped pixel-for-pixel onto chart a
    sv, wv = 0.0, 0.0
    for p in allp:
        ncc, w = p.score(np.zeros((1, 3)), cfg.min_overlap, R=p.R_view[None])
        sv += float(ncc[0]) * float(w[0])
        wv += float(w[0])
    s_view = sv / max(wv, 1e-9)
    margin_v = 1.0 - max(0.0, s_view - med) / max(1e-6, s1 - med)
    # does each camera alone prefer the same solution?
    agree = 1.0
    cam_peaks = {}
    if len(per_cam) > 1:
        for c, arr in per_cam.items():
            hc = int(np.argmax(arr))
            cam_peaks[c] = omegas[hc]
            if np.linalg.norm((omegas[hc] - omega) * dt_min) > 0.2:
                agree *= 0.6
    conf = (float(np.clip((z - 2.0) / 3.0, 0, 1)) * float(np.clip(min(margin_f, margin_v) / 0.30, 0, 1))
            * float(np.clip((s1 - 0.12) / 0.25, 0, 1)) * agree)
    total, axis = spin_from_vector(omega, v_dir)
    # uncertainty from the curvature of the score peak (empirical scaling)
    sigma_rot = 0.012 / max(0.05, s1) / math.sqrt(max(1, len(allp)))
    sigma_rpm = sigma_rot / dt_min / RPM_TO_RADS
    sigma_axis = math.degrees(sigma_rot / max(0.05, np.linalg.norm(omega) * dt_min)) * 1.5
    return SpinEstimate(omega, total, axis, conf, conf >= cfg.measured_threshold, sigma_rpm, sigma_axis,
                        {"peak_ncc": s1, "second_ncc": s2, "view_fixed_ncc": s_view, "margin_view": margin_v, "z": z, "margin_coarse": margin_c, "margin_fine": margin_f,
                         "camera_agreement": agree, "n_pairs": len(allp), "n_charts": sum(len(v) for v in by_cam.values()),
                         "rot_per_interval_deg": math.degrees(float(np.linalg.norm(omega)) * dt_min),
                         "n_hypotheses": len(grid)})
