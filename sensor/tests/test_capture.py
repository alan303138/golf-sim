"""Strobe planning and capture-policy analysis."""
import numpy as np
import pytest

from gs_core.capture import (BurstPolicy, FreeRunPolicy, VisionTriggerPolicy, free_run_dead_time,
                             free_run_miss_probability, plan_strobe, vision_trigger_budget)
from gs_core.geometry import BALL_DIAMETER_M


@pytest.mark.parametrize("v", [15, 40, 54, 75])
def test_spacing_is_1p3_diameters(v):
    p = plan_strobe(v)
    spacing = v * p.config.strobe_interval_us * 1e-6 / BALL_DIAMETER_M
    assert spacing == pytest.approx(1.3, abs=0.02)
    assert 3 <= p.config.pulses_per_exposure <= 8
    # all pulses of the first exposure within the ~250 mm field (after a 0.6 ms acoustic delay)
    travel = v * (0.0006 + (p.config.first_pulse_delay_us + (p.config.pulses_per_exposure - 1)
                            * p.config.strobe_interval_us) * 1e-6)
    assert travel < 0.26
    duty = p.config.strobe_width_us * p.config.pulses_per_exposure / p.config.frame_period_us
    assert duty <= 0.02


def test_default_and_putt_modes():
    d = plan_strobe(None)
    assert not d.single_pulse and 600 <= d.config.strobe_interval_us <= 1200
    p = plan_strobe(3.0)
    assert p.single_pulse and p.config.pulses_per_exposure == 1 and p.config.burst_exposures >= 3


def test_free_run_miss_probability_grows_with_speed():
    c = plan_strobe(None).config
    probs = [free_run_miss_probability(v, c) for v in (3, 15, 40, 75)]
    assert probs[0] < 0.05 and probs[-1] > 0.7
    assert all(a <= b + 1e-9 for a, b in zip(probs, probs[1:]))
    assert free_run_dead_time(c) > 0.7          # exposure ~3 ms vs ~17 ms readout


def test_vision_trigger_budget():
    slow = vision_trigger_budget(200, 75)
    fast = vision_trigger_budget(5000, 75)
    assert slow["ball_travel_m"] > 0.25 and not slow["ok"]     # ball gone before the burst
    assert fast["ok"]


def test_policies_produce_mcu_modes():
    assert BurstPolicy().strobe_config(70).mode == "burst"
    assert FreeRunPolicy().strobe_config(70).mode == "free_run"
    assert VisionTriggerPolicy().strobe_config(70).mode == "burst"
    rest = np.zeros((40, 40)); rest[10:30, 10:30] = 200
    assert VisionTriggerPolicy.departed(np.zeros((40, 40)) + 1, rest)
    assert not VisionTriggerPolicy.departed(rest, rest)
