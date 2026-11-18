"""Calibration from rendered ChArUco views: intrinsics, stereo, world alignment, check."""
import numpy as np
import pytest

from gs_core.calibration import (CharucoSpec, RigCalibration, align_world_from_board, calibrate_intrinsics,
                                 calibrate_stereo, check_with_ball, check_with_board, rotation_angle_deg)
from gs_core.geometry import rotation_matrix
from gs_core.synthetic.board import board_texture, render_board
from gs_core.synthetic.scene import make_rig

SPEC = CharucoSpec(squares_x=8, squares_y=6, square_m=0.030, marker_m=0.022)
BALL_ON_BOARD = (0.12, 0.09)


def _board_poses(rig, n, rng):
    """Board held in front of the pair at 0.35-0.6 m, tilted up to ~35 deg."""
    c = 0.5 * (rig.cam0.center + rig.cam1.center)
    fwd = 0.5 * (rig.cam0.R[2] + rig.cam1.R[2])
    poses = []
    for _ in range(n):
        dist = rng.uniform(0.35, 0.6)
        centre = c + fwd * dist + rng.normal(0, 0.04, 3)
        # board normal facing the cameras, tilted
        R_face = np.stack([rig.cam0.R[0], rig.cam0.R[1], rig.cam0.R[2]], 1)
        tilt = rotation_matrix(np.radians(rng.uniform(-30, 30, 3) * np.array([1, 1, 0.4])))
        R_wb = tilt @ R_face
        half = np.array([SPEC.squares_x * SPEC.square_m / 2, SPEC.squares_y * SPEC.square_m / 2, 0])
        poses.append((R_wb, centre - R_wb @ half))
    return poses


def _mat_pose():
    # board flat on the mat, +x along the target line, ball rest point at BALL_ON_BOARD
    R = np.diag([1.0, -1.0, -1.0])
    t = np.array([-BALL_ON_BOARD[0], BALL_ON_BOARD[1], -0.02134])
    return R, t


@pytest.fixture(scope="module")
def calibrated():
    rig = make_rig()
    rng = np.random.default_rng(3)
    tex = board_texture(SPEC)
    poses = _board_poses(rig, 14, rng)
    imgs = {c.name: [render_board(c, SPEC, R, t, tex, rng=rng) for R, t in poses] for c in rig.pair}
    i0 = calibrate_intrinsics(imgs["cam0"], SPEC, fix_k3=True)
    i1 = calibrate_intrinsics(imgs["cam1"], SPEC, fix_k3=True)
    st = calibrate_stereo(list(zip(imgs["cam0"], imgs["cam1"])), SPEC, i0, i1)
    from gs_core.geometry import CameraModel
    m0 = CameraModel("cam0", i0.K, i0.dist, i0.image_size)
    m1 = CameraModel("cam1", i1.K, i1.dist, i1.image_size)
    Rm, tm = _mat_pose()
    mat0 = render_board(rig.cam0, SPEC, Rm, tm, tex, rng=rng)
    est = align_world_from_board(m0, m1, st, mat0, SPEC, BALL_ON_BOARD, lights_cam0=rig.cam0.to_cam(rig.lights))
    return rig, est, i0, i1, st, tex


def test_intrinsics(calibrated):
    rig, est, i0, i1, st, _ = calibrated
    for intr, true in ((i0, rig.cam0), (i1, rig.cam1)):
        assert intr.rms_px < 0.5
        assert abs(intr.K[0, 0] / true.K[0, 0] - 1) < 0.004          # focal within 0.4 %
        assert np.max(np.abs(intr.K[:2, 2] - true.K[:2, 2])) < 4.0     # principal point within 4 px


def test_stereo_and_world_alignment(calibrated):
    rig, est, i0, i1, st, _ = calibrated
    assert abs(st.baseline_m - rig.baseline_m) < 0.0015                 # baseline within 1.5 mm
    for name in ("cam0", "cam1"):
        e, t = est.cameras[name], rig.cameras[name]
        assert np.linalg.norm(e.center - t.center) < 0.003              # camera position within 3 mm
        assert rotation_angle_deg(e.R @ t.R.T) < 0.3                    # orientation within 0.3 deg
    # what matters for the launch: a ball point maps to the same world coordinates
    X = np.array([[0.0, 0.0, 0.0], [0.15, 0.0, 0.05]])
    for name in ("cam0", "cam1"):
        assert np.max(np.abs(est.cameras[name].project(X) - rig.cameras[name].project(X))) < 3.0
    assert np.linalg.norm(est.lights - rig.lights) < 0.003


def test_save_load_roundtrip(calibrated, tmp_path):
    _, est, *_ = calibrated
    p = tmp_path / "calib.json"
    est.save(p)
    back = RigCalibration.load(p)
    assert np.allclose(back.cam1.R, est.cam1.R) and np.allclose(back.lights, est.lights)


def test_check_detects_movement(calibrated):
    rig, est, _, _, _, tex = calibrated
    rng = np.random.default_rng(9)
    R, t = _board_poses(rig, 1, rng)[0]
    ok = check_with_board(est, render_board(rig.cam0, SPEC, R, t, tex, rng=rng),
                          render_board(rig.cam1, SPEC, R, t, tex, rng=rng), SPEC)
    assert ok.ok and not ok.moved, ok.details
    # cam1 knocked by 0.3 degrees
    c1 = rig.cam1
    dR = rotation_matrix(np.radians([0.0, 0.3, 0.0]))
    moved_cam1 = c1.with_pose(dR @ c1.R, dR @ c1.t)
    bad = check_with_board(est, render_board(rig.cam0, SPEC, R, t, tex, rng=rng),
                           render_board(moved_cam1, SPEC, R, t, tex, rng=rng), SPEC)
    assert bad.moved, bad.details


def test_check_with_ball(calibrated):
    rig, est, *_ = calibrated
    X = np.zeros(3)
    rays = [c.to_world_dir(c.to_cam(X) / np.linalg.norm(c.to_cam(X))) for c in rig.pair]
    d = [float(np.linalg.norm(X - c.center)) for c in rig.pair]
    assert check_with_ball(est, rays[0], rays[1], d[0], d[1]).ok
    # whole unit pushed 8 mm: the ball no longer sits at the origin
    X2 = np.array([0.0, 0.008, 0.0])
    rays2 = [c.to_world_dir(c.to_cam(X2 - [0, 0, 0.006]) / np.linalg.norm(c.to_cam(X2 - [0, 0, 0.006]))) for c in rig.pair]
    assert check_with_ball(est, rays2[0], rays2[1], d[0], d[1]).moved
