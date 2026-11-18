"""Markerless spin estimator on rendered ball images (true ball positions, both cameras).

Lighting: the two LED boards +-140 mm beside the camera pair (weak-moderate
dimple shading), ~450 e-/px per pulse, shot + read noise."""
import numpy as np
import pytest

from gs_core.capture import plan_strobe, pulse_offsets
from gs_core.spin import SpinConfig, estimate_spin, make_chart
from gs_core.spin_model import empirical_spin_rpm, plausible_spin_limit_rpm
from gs_core.synthetic.ball import BallSpec
from gs_core.synthetic.harness import make_world, render_burst
from gs_core.synthetic.scene import PRESETS, ShotTruth


def _charts(scene, truth, cfg, seed=0):
    plan = plan_strobe(truth.speed_mps)
    c = plan.config
    frames, trig = render_burst(scene, truth, c)
    fl = scene.state.flight
    times = [(o + 0.5 * c.strobe_width_us) * 1e-6 for o in pulse_offsets(c)]
    charts = []
    for cam in scene.true_rig.pair:
        raw = frames[cam.name][0].astype(np.float32)
        fg = raw - np.median(raw)
        fg[raw >= 1021] = -2e6
        for t in times:
            p = fl.position(t)
            uv = cam.project(p)
            if 110 < uv[0] < cam.image_size[0] - 110 and 110 < uv[1] < cam.image_size[1] - 110:
                charts.append(make_chart(fg, cam, p, t, scene.true_rig.lights, 1e9, cfg=cfg))
    return charts


@pytest.fixture(scope="module")
def logo_world():
    return make_world(seed=4, marking="logo", calib_error=0.0)


CASES = [("driver", 2700, 0), ("driver", 2700, 15), ("iron7", 7000, -10), ("wedge", 9500, 20), ("wedge", 9500, -20)]


@pytest.mark.parametrize("club,rpm,axis", CASES)
def test_textured_ball(logo_world, club, rpm, axis):
    scene, _ = logo_world
    p = PRESETS[club]
    truth = ShotTruth(p.speed_mps, p.vla_deg, 2.0, rpm, axis)
    cfg = SpinConfig()
    est = estimate_spin(_charts(scene, truth, cfg), truth.velocity / np.linalg.norm(truth.velocity),
                        plausible_spin_limit_rpm(truth.speed_mps, truth.vla_deg), cfg)
    assert est is not None and est.measured, est.details
    assert est.confidence > 0.7
    assert abs(est.total_rpm - rpm) < max(150, 0.03 * rpm), est.details     # 3 % / 150 rpm
    assert abs(est.axis_deg - axis) < 2.0


@pytest.mark.parametrize("club", ["driver", "iron7", "wedge"])
def test_untextured_ball_is_not_reported_as_measured(club):
    """No dimples, no print: the estimator must say it does not know."""
    scene, _ = make_world(seed=5, ball_spec=BallSpec(dimples=False, marking="none"), calib_error=0.0)
    p = PRESETS[club]
    cfg = SpinConfig()
    est = estimate_spin(_charts(scene, p, cfg), p.velocity / np.linalg.norm(p.velocity),
                        plausible_spin_limit_rpm(p.speed_mps, p.vla_deg), cfg)
    assert est is None or (not est.measured and est.confidence < 0.2), est.details


def test_marked_ball_mode():
    scene, _ = make_world(seed=6, marking="dots", calib_error=0.0)
    truth = ShotTruth(40.0, 28.0, 0.0, 9500.0, -20.0)
    cfg = SpinConfig(marked=True)
    est = estimate_spin(_charts(scene, truth, cfg), truth.velocity / np.linalg.norm(truth.velocity), 13000, cfg)
    assert est.measured and est.confidence > 0.8
    assert abs(est.total_rpm - 9500) < 200 and abs(est.axis_deg + 20) < 2.0


def test_empirical_fallback_is_plausible():
    assert 2000 < empirical_spin_rpm(75, 11) < 3500
    assert 5500 < empirical_spin_rpm(54, 16) < 8000
    assert 8000 < empirical_spin_rpm(40, 28) < 11000
    assert empirical_spin_rpm(3, 0) == 0.0
