"""Sign conventions of docs/ARCHITECTURE.md section 3 (hla positive = right = -Y,
spin axis positive = ball curves right)."""
import numpy as np
import pytest

from gs_core.geometry import launch_from_velocity, spin_from_vector, spin_vector, velocity_from_launch
from gs_core.physics import AeroModel


def test_hla_positive_is_right():
    sp, vla, hla = launch_from_velocity(np.array([50.0, -5.0, 10.0]))   # moving toward -Y = right
    assert hla > 0 and vla > 0
    v = velocity_from_launch(60, 12, 5)
    assert v[1] < 0 and v[2] > 0


@pytest.mark.parametrize("speed,vla,hla", [(70, 11, 0), (40, 28, -6), (3, 0, 4)])
def test_launch_roundtrip(speed, vla, hla):
    s, a, h = launch_from_velocity(velocity_from_launch(speed, vla, hla))
    assert s == pytest.approx(speed) and a == pytest.approx(vla) and h == pytest.approx(hla)


def test_backspin_lifts_and_positive_axis_curves_right():
    v = velocity_from_launch(60, 12, 0)
    aero = AeroModel()
    lift = aero.accel(v, spin_vector(3000, 0, v)) - aero.accel(v, None)
    assert lift[2] > 1.0                                    # backspin: Magnus force up
    side = aero.accel(v, spin_vector(3000, 20, v))
    assert side[1] < -1.0                                   # positive axis: force toward -Y (right)
    side = aero.accel(v, spin_vector(3000, -20, v))
    assert side[1] > 1.0


@pytest.mark.parametrize("total,axis", [(2700, 0), (7000, -15), (9500, 20)])
def test_spin_roundtrip(total, axis):
    v = velocity_from_launch(50, 16, 3)
    t, a = spin_from_vector(spin_vector(total, axis, v), v)
    assert t == pytest.approx(total) and a == pytest.approx(axis)


def test_backspin_top_of_ball_moves_back():
    v = velocity_from_launch(60, 0, 0)
    w = spin_vector(3000, 0, v)
    top = np.array([0, 0, 0.021])
    assert np.cross(w, top)[0] < 0                          # surface velocity at the top points toward -X
