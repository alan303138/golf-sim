import math
from gs_common import BallData, ClubData, Shot


def test_roundtrip():
    s = Shot(ball=BallData(67.1, 12.4, -1.8, 2850.0, 4.5), club=ClubData(45.2))
    d = s.to_dict()
    assert d["type"] == "shot" and d["schema"] == 1
    s2 = Shot.from_dict(d)
    assert s2.to_dict() == d
    assert math.isclose(s.ball.back_spin_rpm, 2850 * math.cos(math.radians(4.5)))
