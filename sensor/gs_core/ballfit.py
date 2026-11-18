"""Ball images in a (multi-exposure) frame: candidate search and sub-pixel fit.

A strobe-lit ball is a bright disc on a dark mat. One frame can hold several
images of the same ball (one per strobe pulse), possibly overlapping (slow
ball), a brighter stack of images at the rest position (pulses fired before
impact), and club head / shaft clutter.

1. ``estimate_background``: large-kernel median of a reference frame (removes
   the ball, keeps the smooth mat illumination).
2. ``find_ball_images``: gradient-direction Hough vote for circle centres of
   roughly the expected radius on a 1/4-scale foreground image; interior edges
   of overlapping images vote as well, incoherent club edges do not pile up.
3. ``fit_ball_image``: radial sub-pixel edge search at full resolution, then a
   robust fit of the ball's *cone of sight* (centre direction + angular
   radius) in undistorted ray space, which is exact for a sphere under
   perspective (its image is an ellipse away from the optical axis).
   The edge model is shading-aware: with the LEDs to the side of the lens, the
   threshold contour is the true limb only where the limb is lit; elsewhere it
   is pulled inward toward the terminator. The expected contour is computed
   from a Lambertian sphere and the calibrated LED positions, which removes a
   position-dependent centre bias. Rays that leave the frame, or end inside
   another ball image, are dropped; the fit works on the remaining arc.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .geometry import BALL_RADIUS_M, CameraModel


@dataclass
class BallImage:
    cam: str
    u: float                  # pixel position of the sphere centre
    v: float
    r_px: float               # apparent radius (px)
    ray_c: np.ndarray         # unit direction of the centre, camera frame
    ray_w: np.ndarray         # same, world frame
    beta: float               # angular radius (rad)
    dist_m: float             # camera-to-centre distance implied by the apparent size
    rms_px: float             # edge residual
    arc_frac: float           # fraction of the circumference with usable edges
    level: float              # median foreground inside the disc (DN)
    clipped: bool             # part of the image is outside the frame
    interpolated: bool = False  # interior image of an overlapping chain (position from the chain model)

    @property
    def uv(self) -> np.ndarray:
        return np.array([self.u, self.v])


@dataclass
class FitConfig:
    n_rays: int = 180
    edge_level: float = 0.18          # threshold as a fraction of the single-image level (low: close to the true limb, least shading bias)
    min_arc: float = 0.28
    max_rms_px: float = 1.6
    radius_tol: float = 0.25          # accepted deviation from the expected radius
    vote_min_frac: float = 0.22       # Hough support as a fraction of a full circle
    max_candidates: int = 16
    min_level_dn: float = 25.0        # a ball image must be at least this bright
    decompose_chains: bool = True


# ------------------------------------------------------------- background
def estimate_background(frame: np.ndarray, ball_diameter_px: float = 200.0) -> np.ndarray:
    """Smooth background (mat illumination + black level) with balls removed."""
    f = frame.astype(np.float32)
    small = cv2.resize(f, None, fx=0.125, fy=0.125, interpolation=cv2.INTER_AREA)
    lo, hi = float(small.min()), float(np.percentile(small, 99.9)) + 1e-3
    s8 = np.clip((small - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
    k = int(ball_diameter_px * 0.125 * 2.6) | 1
    med = cv2.medianBlur(s8, k).astype(np.float32) / 255 * (hi - lo) + lo
    # never above the frame's own low percentile where everything is covered (club smear)
    return cv2.resize(med, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_LINEAR)


def expected_radius_px(cam: CameraModel, u: float, v: float) -> float:
    """Apparent ball radius if the ball centre lies on the vertical plane through the target line."""
    o, d = cam.pixel_rays_world(np.array([u, v]))
    t = -o[1] / d[1] if abs(d[1]) > 1e-6 else 0.5
    if not 0.15 < t < 3.0:
        t = float(np.linalg.norm(o))
    dc = cam.pixel_rays_cam(np.array([u, v]))
    return float(cam.fx * BALL_RADIUS_M / (t * dc[2]))


# ----------------------------------------------------------- shading model
def _light_dirs(d: np.ndarray, dist: float, lights_c: np.ndarray | None) -> tuple[np.ndarray, np.ndarray]:
    """Unit directions ball->LED (L,3) and normalised weights (L,). No LEDs: one coaxial light."""
    if lights_c is None or len(lights_c) == 0:
        return -d[None, :], np.ones(1)
    lv = lights_c - dist * d
    ld = np.linalg.norm(lv, axis=1)
    w = 1.0 / ld ** 2
    return lv / ld[:, None], w / w.sum()


def model_intensity(rho: np.ndarray, az_vecs: np.ndarray, d: np.ndarray, l: np.ndarray, w: np.ndarray) -> np.ndarray:
    """Lambertian sphere brightness (1 = facet facing all LEDs) at disc radius
    fraction ``rho`` (broadcast against the azimuth unit vectors (N,3))."""
    a = az_vecs @ l.T                                   # (N, L)
    c = -(d @ l.T)                                      # (L,)
    out = 0.0
    for k in range(len(w)):
        out = out + w[k] * np.clip(rho * a[:, k:k + 1] + np.sqrt(1 - rho ** 2) * c[k], 0, None)
    return out


_RHO = np.linspace(0.55, 1.0, 46)[None, :]


def limb_fraction(az_vecs: np.ndarray, d: np.ndarray, dist: float, lights_c: np.ndarray | None,
                  level: float) -> np.ndarray:
    """Radius (as a fraction of the true limb) of the ``level`` iso-contour of a
    Lambertian sphere, along each in-plane azimuth unit vector (N,3), for a
    ball at ``dist * d`` (camera frame) lit by point sources ``lights_c``.
    NaN where the contour does not exist (too dark along that azimuth)."""
    l, w = _light_dirs(d, dist, lights_c)
    I = model_intensity(_RHO, az_vecs, d, l, w)
    n_r = len(az_vecs)
    rho = _RHO[0]
    above = I >= level
    last = above.shape[1] - 1 - np.argmax(above[:, ::-1], axis=1)
    none = ~above.any(axis=1)
    j = np.clip(last, 0, len(rho) - 2)
    i0, i1 = I[np.arange(n_r), j], I[np.arange(n_r), j + 1]
    frac = np.clip((i0 - level) / np.maximum(i0 - i1, 1e-12), 0, 1)
    out = np.where(last >= len(rho) - 1, 1.0, rho[j] + frac * (rho[j + 1] - rho[j]))
    out[none] = np.nan
    return out


def single_image_level(fg: np.ndarray, cam: CameraModel, u: float, v: float, r: float, d: np.ndarray,
                       lights_c: np.ndarray | None) -> float:
    """Brightness scale E of ONE ball image (image = E * model_intensity).

    Eight small patches at 0.6 r around the centre are each divided by the
    model brightness there; the second-smallest ratio is returned. Where images
    overlap the exposures add up, so the smallest ratios come from patches
    covered by a single image; taking the second smallest tolerates one patch
    darkened by a print."""
    H, W = fg.shape
    th = np.arange(8) * (math.pi / 4) + 0.2
    hw = max(2, int(0.12 * r))
    pts = np.stack([u + 0.6 * r * np.cos(th), v + 0.6 * r * np.sin(th)], 1)
    b1 = np.cross(d, [0.0, 1.0, 0.0])
    b1 /= np.linalg.norm(b1)
    b2 = np.cross(d, b1)
    e = cam.pixel_rays_cam(pts)
    az = (e @ b1)[:, None] * b1 + (e @ b2)[:, None] * b2
    az /= np.linalg.norm(az, axis=1, keepdims=True)
    l, w = _light_dirs(d, BALL_RADIUS_M / max(1e-3, r / cam.fx), lights_c)
    mod = model_intensity(np.full((1, 1), 0.6), az, d, l, w)[:, 0]
    vals = []
    for (x, y), m in zip(pts, mod):
        xi, yi = int(round(x)), int(round(y))
        if xi - hw < 0 or yi - hw < 0 or xi + hw >= W or yi + hw >= H or m < 0.2:
            continue
        vals.append(float(np.median(fg[yi - hw:yi + hw + 1, xi - hw:xi + hw + 1])) / m)
    if len(vals) < 3:
        return 0.0
    vals.sort()
    return vals[1]


# --------------------------------------------------------------- single fit
def _edges(fg: np.ndarray, u: float, v: float, r: float, cfg: FitConfig, level: float):
    """Sub-pixel threshold crossings along radial rays. Returns (pts (M,2), ok mask over rays)."""
    th = np.arange(cfg.n_rays) * (2 * math.pi / cfg.n_rays)
    s = np.arange(0.62 * r, 1.42 * r, 0.5, dtype=np.float32)
    mx = (u + np.cos(th)[:, None] * s[None, :]).astype(np.float32)
    my = (v + np.sin(th)[:, None] * s[None, :]).astype(np.float32)
    prof = cv2.remap(fg, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=-1e6)
    off = prof < -1e5
    T = cfg.edge_level * level
    above = prof >= T
    n_s = len(s)
    last = n_s - 1 - np.argmax(above[:, ::-1], axis=1)
    j = np.clip(last, 6, n_s - 2)
    rows = np.arange(cfg.n_rays)
    guard = int(0.12 * r / 0.5)
    ok = above.any(axis=1) & (last >= 6) & (last < n_s - 1 - guard)
    # a solid run of foreground just inside the edge, nothing off-frame around it
    inside = np.stack([above[rows, np.clip(j - k, 0, n_s - 1)] for k in range(1, 7)], 1).mean(1)
    ok &= inside >= 0.99
    ok &= ~off[rows, np.clip(j + 2, 0, n_s - 1)] & ~off[rows, j]
    f0, f1 = prof[rows, j], prof[rows, j + 1]
    frac = np.clip((f0 - T) / np.maximum(f0 - f1, 1e-6), 0, 1)
    se = s[j] + frac * 0.5
    pts = np.stack([u + np.cos(th) * se, v + np.sin(th) * se], 1)
    return pts, ok, bool(off.any())


def _consensus(e: np.ndarray, d: np.ndarray, beta: float, fx: float, lights_c: np.ndarray | None,
               cfg: FitConfig) -> np.ndarray:
    """RANSAC circle on the edge rays (tangent plane at ``d``), radius constrained
    to the expected one; returns 0/1 inlier weights. Needed when a large part of
    the edges belongs to neighbouring, overlapping ball images."""
    b1 = np.cross(d, [0.0, 1.0, 0.0])
    b1 /= np.linalg.norm(b1)
    b2 = np.cross(d, b1)
    ez = e @ d
    px, py = (e @ b1) / ez, (e @ b2) / ez
    rad = np.hypot(px, py)
    az = (px[:, None] * b1 + py[:, None] * b2) / rad[:, None]
    rho = limb_fraction(az, d, BALL_RADIUS_M / math.sin(beta), lights_c, cfg.edge_level)
    good = np.isfinite(rho)
    rho = np.where(good, rho, 1.0)
    x, y = px / rho, py / rho                 # shading-corrected: points on the true limb circle
    n = len(x)
    rng = np.random.default_rng(n)
    idx = np.stack([rng.permutation(n)[:3] for _ in range(80)])
    x1, y1, x2, y2, x3, y3 = x[idx[:, 0]], y[idx[:, 0]], x[idx[:, 1]], y[idx[:, 1]], x[idx[:, 2]], y[idx[:, 2]]
    a = 2 * (x2 - x1); b = 2 * (y2 - y1); c = x2 ** 2 + y2 ** 2 - x1 ** 2 - y1 ** 2
    a2 = 2 * (x3 - x1); b2_ = 2 * (y3 - y1); c2 = x3 ** 2 + y3 ** 2 - x1 ** 2 - y1 ** 2
    det = a * b2_ - a2 * b
    det = np.where(np.abs(det) < 1e-12, 1e-12, det)
    cx, cy = (c * b2_ - c2 * b) / det, (a * c2 - a2 * c) / det
    rr = np.hypot(x1 - cx, y1 - cy)
    r0 = math.tan(beta)
    valid = np.abs(rr / r0 - 1) < cfg.radius_tol
    dist = np.abs(np.hypot(x[None, :] - cx[:, None], y[None, :] - cy[:, None]) - rr[:, None])
    inl = (dist < 1.2 / fx) & good[None, :]
    score = inl.sum(axis=1) * valid
    return inl[int(np.argmax(score))].astype(float)


def fit_ball_image(fg: np.ndarray, cam: CameraModel, u: float, v: float, r: float,
                   lights_c: np.ndarray | None = None, cfg: FitConfig | None = None) -> BallImage | None:
    """Refine one ball image starting from an approximate centre/radius (px).
    ``fg`` is the background-subtracted, lightly smoothed frame (float32)."""
    cfg = cfg or FitConfig()
    H, W = fg.shape
    res = None
    for it in range(3):
        if not (-0.6 * r < u < W + 0.6 * r and -0.6 * r < v < H + 0.6 * r) or not 8 < r < 400:
            return None
        d = cam.pixel_rays_cam(np.array([u, v]))
        level = single_image_level(fg, cam, u, v, r, d, lights_c)
        if level < cfg.min_level_dn:
            return None
        pts, ok, clipped = _edges(fg, u, v, r, cfg, level)
        if ok.sum() < cfg.min_arc * cfg.n_rays * 0.8:
            return None
        e = cam.pixel_rays_cam(pts[ok])
        beta = math.atan(r / cam.fx * d[2] ** 2) if it == 0 else res[1]
        w = _consensus(e, d, beta, cam.fx, lights_c, cfg)
        w0 = w.copy()
        if w.sum() < cfg.min_arc * cfg.n_rays * 0.7:
            return None
        for gn in range(4):
            b1 = np.cross(d, [0.0, 1.0, 0.0])
            b1 /= np.linalg.norm(b1)
            b2 = np.cross(d, b1)
            ez = e @ d
            px, py = (e @ b1) / ez, (e @ b2) / ez
            rad = np.hypot(px, py)
            az = (px[:, None] * b1 + py[:, None] * b2) / rad[:, None]
            rho = limb_fraction(az, d, BALL_RADIUS_M / math.sin(beta), lights_c, cfg.edge_level)
            good = np.isfinite(rho)
            rho = np.where(good, rho, 1.0)
            # residual_i = rad_i - (cx*cos + cy*sin) - tan(beta)*rho_i   (linear in cx, cy, tan beta)
            A = np.stack([px / rad, py / rad, rho], 1)
            ww = w * good
            sol, *_ = np.linalg.lstsq(A * ww[:, None], rad * ww, rcond=None)
            resid = rad - A @ sol
            sig = 1.4826 * np.median(np.abs(resid[ww > 0])) + 0.15 / cam.fx
            w = np.where(np.abs(resid) < 2.5 * sig, 1.0, 0.0) * (w0 if gn < 2 else 1.0)
            d = d + sol[0] * b1 + sol[1] * b2
            d /= np.linalg.norm(d)
            beta = math.atan(sol[2])
        inl = (w * good) > 0
        rms = float(np.sqrt(np.mean(resid[inl] ** 2)) * cam.fx) if inl.any() else 1e9
        res = (d, beta, rms, float(inl.sum()) / cfg.n_rays, level, clipped)
        uv = cam.project_cam(d)
        u, v = float(uv[0]), float(uv[1])
        r = float(cam.fx * math.tan(beta) / d[2] ** 2)
    d, beta, rms, arc, level, clipped = res
    if arc < cfg.min_arc or rms > cfg.max_rms_px or beta <= 0:
        return None
    return BallImage(cam.name, u, v, r, d, cam.to_world_dir(d), beta, BALL_RADIUS_M / math.sin(beta),
                     rms, arc, level, clipped)


# ------------------------------------------------------------ candidates
def prepare_foreground(frame: np.ndarray, background: np.ndarray) -> np.ndarray:
    fg = frame.astype(np.float32) - background
    return cv2.GaussianBlur(fg, (0, 0), 1.0)


def find_ball_images(fg: np.ndarray, cam: CameraModel, lights_c: np.ndarray | None = None,
                     cfg: FitConfig | None = None) -> list[BallImage]:
    """All ball images in a background-subtracted frame (see module docstring)."""
    cfg = cfg or FitConfig()
    H, W = fg.shape
    sc = 4
    small = cv2.resize(fg, (W // sc, H // sc), interpolation=cv2.INTER_AREA)
    small = cv2.GaussianBlur(small, (0, 0), 1.0)
    bright = small[small > cfg.min_level_dn]
    if bright.size < 50:
        return []
    lvl = float(np.percentile(bright, 60))
    gx = cv2.Sobel(small, cv2.CV_32F, 1, 0, ksize=3) / 8
    gy = cv2.Sobel(small, cv2.CV_32F, 0, 1, ksize=3) / 8
    mag = np.hypot(gx, gy)
    ys, xs = np.nonzero(mag > 0.10 * lvl)
    if len(xs) < 20:
        return []
    r0 = expected_radius_px(cam, W / 2, H / 2) / sc
    rr = np.arange(r0 * (1 - cfg.radius_tol), r0 * (1 + cfg.radius_tol) + 1, 1.0)
    ux, uy = gx[ys, xs] / mag[ys, xs], gy[ys, xs] / mag[ys, xs]
    pad = int(r0 * 0.8)
    acc = np.zeros((H // sc + 2 * pad, W // sc + 2 * pad), np.float32)
    cx = np.rint(xs[:, None] + ux[:, None] * rr[None, :]).astype(np.intp) + pad
    cy = np.rint(ys[:, None] + uy[:, None] * rr[None, :]).astype(np.intp) + pad
    okv = (cx >= 0) & (cx < acc.shape[1]) & (cy >= 0) & (cy < acc.shape[0])
    np.add.at(acc, (cy[okv], cx[okv]), 1.0)
    acc = cv2.boxFilter(acc, -1, (5, 5), normalize=False) / 5.0
    acc = cv2.GaussianBlur(acc, (0, 0), 1.0)
    thr = cfg.vote_min_frac * 2 * math.pi * r0
    cands: list[tuple[float, float, float]] = []
    work = acc.copy()
    supp = max(3, int(0.22 * r0))
    for _ in range(cfg.max_candidates):
        _, mv, _, ml = cv2.minMaxLoc(work)
        if mv < thr:
            break
        cands.append(((ml[0] - pad + 0.5) * sc, (ml[1] - pad + 0.5) * sc, mv))
        cv2.circle(work, ml, supp, 0, -1)
    out: list[BallImage] = []
    for u, v, _ in cands:
        if any(math.hypot(o.u - u, o.v - v) < 0.3 * o.r_px for o in out):
            continue                       # already explained by an accepted image
        b = fit_ball_image(fg, cam, u, v, r0 * sc, lights_c, cfg)
        if b is None:
            continue
        if abs(b.r_px / expected_radius_px(cam, b.u, b.v) - 1) > cfg.radius_tol:
            continue
        dup = False
        for i, o in enumerate(out):
            if math.hypot(o.u - b.u, o.v - b.v) < 0.12 * b.r_px:
                dup = True
                if (b.arc_frac, -b.rms_px) > (o.arc_frac, -o.rms_px):
                    out[i] = b
                break
        if not dup:
            out.append(b)
    if out:
        ref = float(np.median(sorted((o.level for o in out), reverse=True)[:3]))
        out = [o for o in out if o.level > 0.45 * ref]
    if cfg.decompose_chains:
        out = out + decompose_chains(fg, cam, out, lights_c, cfg)
    return out


def _disc_flux_factor(cam: CameraModel, u: float, v: float, r: float, lights_c: np.ndarray | None) -> float:
    """Mean of the normalised Lambertian model over the disc (flux = E * pi r^2 * factor)."""
    d = cam.pixel_rays_cam(np.array([u, v]))
    l, w = _light_dirs(d, BALL_RADIUS_M / max(1e-3, r / cam.fx), lights_c)
    b1 = np.cross(d, [0.0, 1.0, 0.0])
    b1 /= np.linalg.norm(b1)
    b2 = np.cross(d, b1)
    th = np.linspace(0, 2 * math.pi, 24, endpoint=False)
    az = np.cos(th)[:, None] * b1 + np.sin(th)[:, None] * b2
    rho = np.linspace(0.02, 0.98, 25)[None, :]
    I = model_intensity(rho, az, d, l, w)
    return float(np.sum(I * rho) / np.sum(np.broadcast_to(rho, I.shape)))


def _local_level(fg: np.ndarray, cam: CameraModel, c: np.ndarray, r: float, lights_c: np.ndarray | None) -> float:
    """Median of fg / model over a disc of 0.5 r around ``c`` (brightness scale E there)."""
    H, W = fg.shape
    x0, y0 = int(c[0] - 0.5 * r), int(c[1] - 0.5 * r)
    x1, y1 = int(c[0] + 0.5 * r) + 1, int(c[1] + 0.5 * r) + 1
    if x0 < 0 or y0 < 0 or x1 > W or y1 > H:
        return 0.0
    d = cam.pixel_rays_cam(c)
    l, w = _light_dirs(d, BALL_RADIUS_M / max(1e-3, r / cam.fx), lights_c)
    m0 = float(model_intensity(np.zeros((1, 1)), (np.cross(d, [0.0, 1.0, 0.0]) / np.linalg.norm(np.cross(d, [0.0, 1.0, 0.0])))[None, :], d, l, w)[0, 0])
    return float(np.median(fg[y0:y1, x0:x1])) / max(m0, 1e-3)


def decompose_chains(fg: np.ndarray, cam: CameraModel, found: list[BallImage], lights_c: np.ndarray | None,
                     cfg: FitConfig) -> list[BallImage]:
    """Interior images of chains of overlapping ball images (slow ball or too
    short strobe interval) that the edge-based fit cannot isolate.

    Strobe pulses are equally spaced in time, so the images of one chain are
    equally spaced along a line (constant image-plane velocity over a few ms;
    the perspective error is < 0.2 px over a chain of ~2 ball diameters).
    Anchors are two neighbouring images that were fitted individually (usually
    the free end and its neighbour); the chain is extended from them in steps
    of their spacing while the image brightness, divided by the Lambertian
    model, still shows at least one exposure there, and gaps between anchors
    that are multiples of the step are filled. The extra images are flagged
    ``interpolated`` (their position comes from the chain model, not from
    their own edges)."""
    if len(found) < 2:
        return []
    r = float(np.median([o.r_px for o in found]))
    E = float(np.median([o.level for o in found]))
    extra: list[BallImage] = []
    pts = [o for o in found if not o.interpolated]
    best = None
    for i in range(len(pts)):
        for j in range(len(pts)):
            if i == j:
                continue
            d = math.hypot(pts[i].u - pts[j].u, pts[i].v - pts[j].v)
            if 0.25 * r < d < 1.9 * r and (best is None or d < best[0]):
                best = (d, pts[i], pts[j])
    if best is None:
        return []
    step_len, a, b = best
    step = np.array([b.u - a.u, b.v - a.v])
    known = [np.array([o.u, o.v]) for o in found]

    def explained(c: np.ndarray) -> bool:
        return any(np.linalg.norm(k - c) < 0.35 * step_len for k in known)

    def add(c: np.ndarray) -> None:
        d = cam.pixel_rays_cam(c)
        beta = math.atan(r / cam.fx * d[2] ** 2)
        extra.append(BallImage(cam.name, float(c[0]), float(c[1]), r, d, cam.to_world_dir(d), beta, float("nan"),
                               float(np.mean([a.rms_px, b.rms_px])), 0.0, E, False, interpolated=True))
        known.append(c)

    for direction in (1.0, -1.0):
        for k in range(1, 9):
            c = np.array([a.u, a.v]) + direction * k * step
            if explained(c):
                continue
            # the chain continues only while there is at least one exposure's worth of ball light
            if _local_level(fg, cam, c, r, lights_c) < 0.55 * E:
                break
            add(c)
    return extra
