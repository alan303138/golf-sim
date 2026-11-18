"""End-to-end accuracy on rendered multi-exposure frames.

Everything is synthetic: renderer -> detection -> stereo -> association ->
trajectory -> spin, with sensor noise (~450 e-/px per pulse, shot + read
noise), the LED geometry of the hardware design, a club head in view, the
ball 0-6 mm off the calibrated origin, and a calibration with realistic
errors (``perturbed_rig``: focal 0.1 %, principal point 0.7 px, relative
rotation 0.015 deg, world yaw 0.2 deg, origin 1 mm ... 1 sigma).

Tolerances are stated per quantity below. They are what the pipeline must
meet on synthetic data; synthetic success does NOT prove real-world accuracy
(real dimple contrast, cover BRDF, focus, vibration, calibration quality).
Run ``python -m gs_core.tools.accuracy`` for the full per-shot-type table.
"""
import pytest

from gs_core.synthetic.harness import run_shot
from gs_core.synthetic.scene import PRESETS, ShotTruth

TOL_SPEED = lambda v: max(0.4, 0.008 * v)   # m/s
TOL_VLA = 0.6                               # deg
TOL_HLA = 0.8                               # deg (world-alignment error enters 1:1)
TOL_SPIN = lambda rpm: max(150.0, 0.04 * rpm)
TOL_AXIS = 3.0                              # deg

FULL = [
    ("driver", ShotTruth(75.0, 11.0, 0.0, 2700.0, 0.0)),
    ("driver_push_fade", ShotTruth(72.0, 10.0, 8.0, 2900.0, 20.0)),
    ("iron7", ShotTruth(54.0, 16.0, 0.0, 7000.0, 0.0)),
    ("iron7_pull_draw", ShotTruth(54.0, 17.0, -8.0, 6800.0, -20.0)),
    ("wedge", ShotTruth(40.0, 28.0, 0.0, 9500.0, 0.0)),
    ("wedge_axis", ShotTruth(40.0, 29.0, 3.0, 9200.0, 12.0)),
]


@pytest.mark.slow
@pytest.mark.parametrize("name,truth", FULL, ids=[n for n, _ in FULL])
def test_full_shots(world, name, truth):
    scene, est = world
    o = run_shot(scene, est, truth, seed=sum(map(ord, name)))
    assert o.result.shot is not None, o.result.reason
    e = o.err
    assert abs(e["speed"]) < TOL_SPEED(truth.speed_mps), e
    assert abs(e["vla"]) < TOL_VLA, e
    assert abs(e["hla"]) < TOL_HLA, e
    q = o.result.shot.quality
    assert q.spin_measured, o.result.spin.details if o.result.spin else "no spin estimate"
    assert abs(e["spin"]) < TOL_SPIN(truth.total_spin_rpm), e
    assert abs(e["axis"]) < TOL_AXIS, e


@pytest.mark.slow
def test_chip(world):
    scene, est = world
    truth = ShotTruth(15.0, 30.0, 2.0, 3500.0, 5.0)
    o = run_shot(scene, est, truth, seed=7)
    e = o.err
    assert o.result.shot is not None, o.result.reason
    assert abs(e["speed"]) < 0.3 and abs(e["vla"]) < 0.8 and abs(e["hla"]) < 1.0, e
    if o.result.shot.quality.spin_measured:
        assert abs(e["spin"]) < TOL_SPIN(3500), e


@pytest.mark.slow
@pytest.mark.parametrize("hla", [0.0, 4.0])
def test_putt(world, hla):
    scene, est = world
    truth = ShotTruth(3.0, 0.0, hla, rolling=True, club=False)
    o = run_shot(scene, est, truth, seed=11)
    e = o.err
    assert o.result.shot is not None, o.result.reason
    assert o.result.launch.trajectory.model == "rolling"
    assert abs(e["speed"]) < 0.08 and abs(e["hla"]) < 1.0 and abs(e["vla"]) < 1.0, e
    assert not o.result.shot.quality.spin_measured


@pytest.mark.slow
def test_wrong_speed_hint_overlapping_images(world):
    """Wedge struck while the strobe is still planned for a driver (images 0.7 D
    apart, overlapping). Launch still within tolerance; spin may fall back."""
    scene, est = world
    truth = PRESETS["wedge"]
    o = run_shot(scene, est, truth, speed_hint=75.0, seed=3)
    e = o.err
    assert o.result.shot is not None, o.result.reason
    assert abs(e["speed"]) < TOL_SPEED(40) and abs(e["vla"]) < TOL_VLA and abs(e["hla"]) < 1.5, e


@pytest.mark.slow
def test_plain_ball_falls_back_to_estimate():
    """A ball without visible texture: launch still measured, spin from the model, flagged."""
    from gs_core.synthetic.ball import BallSpec
    from gs_core.synthetic.harness import make_world
    scene, est = make_world(seed=2, ball_spec=BallSpec(dimples=False, marking="none"))
    o = run_shot(scene, est, PRESETS["iron7"], seed=1)
    s = o.result.shot
    assert s is not None and not s.quality.spin_measured
    assert abs(o.err["speed"]) < TOL_SPEED(54)
    assert 5000 < s.ball.total_spin_rpm < 9000 and s.ball.spin_axis_deg == 0.0
