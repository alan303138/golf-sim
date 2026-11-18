"""Stereo triangulation, pulse association and trajectory fit (geometric, no rendering)."""
import math

import numpy as np
import pytest

from gs_core.ballfit import BallImage
from gs_core.calibration import midpoint_triangulate
from gs_core.geometry import BALL_RADIUS_M, velocity_from_launch
from gs_core.synthetic.scene import make_rig
from gs_core.physics import integrate
from gs_core.track import GRAVITY, Slot, associate, fit_trajectory, stereo_match

RIG = make_rig()


def _img(cam, X, noise_px=0.0, rng=None):
    uv = cam.project(X) + (rng.normal(0, noise_px, 2) if rng is not None else 0)
    d = cam.pixel_rays_cam(uv)
    dist = float(np.linalg.norm(X - cam.center))
    beta = math.asin(BALL_RADIUS_M / dist)
    return BallImage(cam.name, float(uv[0]), float(uv[1]), cam.fx * math.tan(beta), d, cam.to_world_dir(d), beta,
                     dist, 0.5, 0.9, 300.0, False)


def _slots(n_exp, offsets_us, period_us, delay_us=0):
    return [Slot(e, k, (delay_us + e * period_us + o + 5) * 1e-6) for e in range(n_exp) for k, o in enumerate(offsets_us)]


def _balls(v, slots, rest=np.zeros(3), t_imp=-0.0006, noise=0.15, rng=None, keep=None):
    rng = rng or np.random.default_rng(0)
    per_exp = {}
    for i, s in enumerate(slots):
        if keep is not None and i not in keep:
            continue
        tau = s.t - t_imp
        X = rest + integrate(np.zeros(3), v, None, np.array([tau]))[0][0] if tau > 0 else rest
        per_exp.setdefault(s.exposure, []).append(X)
    out = []
    for e, Xs in per_exp.items():
        a = [_img(RIG.cam0, X, noise, rng) for X in Xs]
        b = [_img(RIG.cam1, X, noise, rng) for X in Xs]
        rng.shuffle(b)
        out += stereo_match(a, b, RIG, e)
    return out


def test_triangulation_exact():
    X = np.array([0.12, 0.01, 0.03])
    a, b = _img(RIG.cam0, X), _img(RIG.cam1, X)
    P, gap = midpoint_triangulate(RIG.cam0.center, a.ray_w, RIG.cam1.center, b.ray_w)
    assert np.linalg.norm(P - X) < 1e-9 and gap < 1e-9


def test_triangulation_noise_depth_error():
    """0.2 px centre noise -> depth error ~0.3 mm at 0.5 m with a 160 mm baseline."""
    rng = np.random.default_rng(1)
    X = np.array([0.1, 0.0, 0.02])
    errs = []
    for _ in range(300):
        a, b = _img(RIG.cam0, X, 0.2, rng), _img(RIG.cam1, X, 0.2, rng)
        P, _ = midpoint_triangulate(RIG.cam0.center, a.ray_w, RIG.cam1.center, b.ray_w)
        errs.append(P - X)
    errs = np.array(errs)
    assert np.std(errs[:, 1]) < 0.0006 and np.std(errs[:, 0]) < 0.0002


@pytest.mark.parametrize("speed,vla,hla", [(75, 11, 0), (54, 16, 8), (40, 28, -8)])
def test_association_and_launch(speed, vla, hla):
    v = velocity_from_launch(speed, vla, hla)
    offs = [30 + k * 740 for k in range(5)]
    slots = _slots(2, offs, 19000)
    balls = _balls(v, slots)
    L = associate(balls, slots, RIG, np.zeros(3))
    assert L is not None and L.n_clutter == 0
    assert abs(L.speed_mps - speed) < 0.3 and abs(L.vla_deg - vla) < 0.3 and abs(L.hla_deg - hla) < 0.4
    # reported 1-sigma is honest: errors within 4 sigma
    assert abs(L.speed_mps - speed) < 4 * L.sigma_speed + 0.05
    for b in L.balls:                                   # every image assigned to its true pulse
        assert np.linalg.norm(b.p - balls[0].p) >= 0 and slots[b.slot].exposure == b.exposure


def test_association_rejects_clutter_and_rest_ghost():
    v = velocity_from_launch(60, 14, 2)
    offs = [30 + k * 900 for k in range(5)]
    slots = _slots(1, offs, 19000)
    # trigger early: first pulse fires before impact -> ghost at the rest position
    balls = _balls(v, slots, t_imp=0.0005)
    clutter = [_img(RIG.cam0, np.array([-0.06, 0.03, 0.02])), _img(RIG.cam0, np.array([0.02, 0.05, 0.09]))]
    clutter_b = [_img(RIG.cam1, np.array([-0.06, 0.03, 0.02])), _img(RIG.cam1, np.array([0.02, 0.05, 0.09]))]
    balls += stereo_match(clutter, clutter_b, RIG, 0)
    L = associate(balls, slots, RIG, np.zeros(3))
    assert L is not None
    assert L.n_rest >= 1 and L.n_clutter == 2
    assert abs(L.speed_mps - 60) < 0.3 and abs(L.hla_deg - 2) < 0.4
    assert abs(L.trajectory.t_impact - 0.0005) < 0.0001


def test_ball_leaves_frame_and_second_exposure():
    """6 pulses planned, the ball leaves the view after a few; exposure 1 is empty."""
    from gs_core.track import _visible
    v = velocity_from_launch(75, 10, 0)
    offs = [30 + k * 1000 for k in range(6)]
    slots = _slots(2, offs, 19000)
    pos = np.array([v * (s.t + 0.0006) + 0.5 * GRAVITY * (s.t + 0.0006) ** 2 for s in slots])
    keep = {i for i in range(len(slots)) if _visible(RIG, pos[i], 30.0)}
    assert 3 <= len(keep) < 6
    balls = _balls(v, slots, keep=keep)
    L = associate(balls, slots, RIG, np.zeros(3))
    assert L is not None and sorted(b.slot for b in L.balls) == sorted(keep)
    assert abs(L.speed_mps - 75) < 0.3


def test_flight_split_across_exposures():
    """Slow ball: images in two consecutive exposures, joined in one fit."""
    v = velocity_from_launch(12, 30, 3)
    offs = [30 + k * 4000 for k in range(3)]
    slots = _slots(2, offs, 13000)
    balls = _balls(v, slots)
    L = associate(balls, slots, RIG, np.zeros(3))
    assert L is not None and {b.exposure for b in L.balls} == {0, 1}
    assert abs(L.speed_mps - 12) < 0.1 and abs(L.vla_deg - 30) < 0.3


def test_putt_rolling_model():
    v = velocity_from_launch(3.0, 0, 1.5)
    slots = _slots(5, [50], 17000)
    rng = np.random.default_rng(2)
    per = []
    for s in slots:
        tau = s.t + 0.0006
        X = v * tau - 0.3 * v / 3.0 * tau * tau          # 0.6 m/s^2 rolling deceleration
        per.append((s.exposure, X))
    balls = []
    for e, X in per:
        balls += stereo_match([_img(RIG.cam0, X, 0.15, rng)], [_img(RIG.cam1, X, 0.15, rng)], RIG, e)
    L = associate(balls, slots, RIG, np.zeros(3))
    assert L is not None and L.trajectory.model == "rolling"
    assert abs(L.speed_mps - 3.0) < 0.03 and abs(L.hla_deg - 1.5) < 0.5


def test_fit_recovers_launch_velocity_with_drag():
    """Noise-free points from the full flight model (drag, no spin): the fit's
    gravity + drag model returns the velocity at t = 0 within 0.01 m/s."""
    from gs_core.physics import integrate
    v = velocity_from_launch(75, 11, 0)
    t = 0.0006 + np.arange(4) * 0.00074
    P, _ = integrate(np.zeros(3), v, None, t)
    tr = fit_trajectory(P, t, RIG)
    assert np.linalg.norm(tr.velocity(0.0) - v) < 0.01
    assert np.sqrt(tr.cov[3, 3]) < 0.2
