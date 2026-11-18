"""Ball image detection and sub-pixel fit on rendered multi-exposure frames."""
import numpy as np

from gs_core.ballfit import estimate_background, find_ball_images, prepare_foreground
from gs_core.capture import plan_strobe, pulse_offsets
from gs_core.pipeline import detect_rest
from gs_core.synthetic.harness import render_burst, render_rest
from gs_core.synthetic.scene import PRESETS


def _truth_px(scene, cam, cfg, trigger_delay=0.0006):
    fl = scene.state.flight
    return np.array([cam.project(fl.position((o + 0.5 * cfg.strobe_width_us) * 1e-6)) for o in pulse_offsets(cfg)])


def _detect(scene, truth, hint, cfg=None):
    plan = plan_strobe(hint)
    frames, _ = render_burst(scene, truth, plan.config)
    out = {}
    for cam in scene.true_rig.pair:
        raw = frames[cam.name][0]
        fg = prepare_foreground(raw, estimate_background(raw))
        out[cam.name] = (find_ball_images(fg, cam, cam.to_cam(scene.true_rig.lights), cfg), _truth_px(scene, cam, plan.config))
    return out


def _match(found, truth_px):
    errs = []
    for t in truth_px:
        d = [np.hypot(b.u - t[0], b.v - t[1]) for b in found]
        errs.append(min(d) if d else np.inf)
    return np.array(errs)


def test_separated_images_subpixel(world_exact):
    """Driver: 4 separated images per frame, club head and shaft in view.

    Centres within 2 px (the first image can touch the club head following the
    ball, which disturbs part of its edge), median < 0.8 px."""
    scene, _ = world_exact
    for name, (found, tpx) in _detect(scene, PRESETS["driver"], 75).items():
        e = _match(found, tpx)
        assert len(found) == len(tpx), name                 # no club/shaft false positives
        assert np.all(e < 2.0), (name, e)
        assert np.median(e) < 0.8, (name, e)


def test_partially_overlapping_images(world_exact):
    """Wedge (40 m/s) with a strobe planned for 75 m/s: images 0.7 D apart (overlapping).

    Achieved (and asserted): every image found; the free end image < 1.5 px;
    interior images, whose edges are hidden along the chain, only to ~6 px.
    The trajectory fit down-weights them (see test_e2e overlap case)."""
    scene, _ = world_exact
    for name, (found, tpx) in _detect(scene, PRESETS["wedge"], 75).items():
        e = _match(found, tpx)
        assert np.all(e < 6.0), (name, e)
        assert e[-1] < 1.5, (name, e)


def test_chain_decomposition(world_exact):
    """Chip (15 m/s) with a strobe planned for 25 m/s: 5 images ~0.78 D apart.

    Achieved (and asserted): every image found within 10 px; at least 3 within
    1.5 px. Interior images of a chain are located poorly ALONG the chain
    (their leading/trailing edges are hidden by the neighbours): 3-8 px."""
    scene, _ = world_exact
    for name, (found, tpx) in _detect(scene, PRESETS["chip"], 25).items():
        e = _match(found, tpx)
        assert np.all(e < 10.0), (name, e)
        assert np.sum(e < 1.5) >= 3, (name, e)


def test_rest_detection(world):
    scene, est = world
    for rest in (np.zeros(3), np.array([0.01, -0.02, 0.0])):
        X, info = detect_rest(render_rest(scene, rest), est)
        assert X is not None
        assert np.linalg.norm(X - rest) < 0.004, info        # calibration error dominates (~1-2 mm)
    scene.remove_ball()
    X, _ = detect_rest({c.name: scene.render(c.name, [0.0], 10e-6) for c in scene.true_rig.pair}, est)
    assert X is None
    scene.place_ball()
