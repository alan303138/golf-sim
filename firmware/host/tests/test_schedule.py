import dataclasses

import pytest

from gs_common.interfaces import StrobeConfig
from gs_mcu import errors as E
from gs_mcu import schedule as S
from gs_mcu.protocol import Options


def cfg(**kw):
    return dataclasses.replace(StrobeConfig(), **kw)


def replay(sched):
    """Replay the PIO words: returns (cam intervals, strobe intervals, total cycles) in cycles."""
    t, cam, st = 0, 0, 0
    cam_iv, st_iv, cam_on, st_on = [], [], None, None
    for w in sched.words:
        ncam, nst = w & 1, (w >> 1) & 1
        if ncam and not cam:
            cam_on = t
        if cam and not ncam:
            cam_iv.append((cam_on, t))
        if nst and not st:
            st_on = t
        if st and not nst:
            st_iv.append((st_on, t))
        cam, st = ncam, nst
        t += S.wave_word_cycles(w)
    return cam_iv, st_iv, t


def test_default_offsets_and_starts():
    s = S.build(StrobeConfig())
    assert s.exposure_len_us == 2910
    assert s.pulse_offsets_us == [50, 750, 1450, 2150, 2850]
    assert s.exposure_start_rel_us == [0, 16700]
    assert s.lockout_eff_us == 500_000
    cam, st, total = replay(s)
    assert total == s.span_cycles
    assert len(cam) == 2 and len(st) == 10
    for e in range(2):
        assert cam[e][0] == s.exposure_start_rel_us[e] * 125
        assert cam[e][1] - cam[e][0] == 2910 * 125
        for p in range(5):
            on, off = st[e * 5 + p]
            assert on == cam[e][0] + s.pulse_offsets_us[p] * 125
            assert off - on == 10 * 125          # width exact to the clock cycle
    assert s.words[-1] & 4 and not any(w & 4 for w in s.words[:-1])


def test_trigger_delay():
    s = S.build(cfg(trigger_delay_us=1234))
    cam, _, _ = replay(s)
    assert cam[0][0] == 1234 * 125
    assert s.exposure_start_rel_us == [1234, 1234 + 16700]


def test_free_run_frame_exact():
    s = S.build(cfg(mode="free_run"))
    cam, st, total = replay(s)
    assert total == 16700 * 125 and not any(w & 4 for w in s.words)
    assert cam[0][0] == 0 and len(st) == 5 and s.n_exposures == 1
    assert S.build(cfg(mode="free_run", frame_period_us=2910 + 100)).span_cycles == 3010 * 125
    with pytest.raises(E.McuError) as ei:
        S.build(cfg(mode="free_run", frame_period_us=2910 + 99))
    assert ei.value.code == E.LIMIT_TIMING and ei.value.detail == 8


@pytest.mark.parametrize("w,ok", [(5, True), (20, True), (4, False), (21, False), (100, False), (0, False)])
def test_width_boundaries_rejected_not_clipped(w, ok):
    if ok:
        assert S.build(cfg(strobe_width_us=w)).pulse_offsets_us
    else:
        with pytest.raises(E.McuError) as ei:
            S.build(cfg(strobe_width_us=w))
        assert ei.value.code == E.LIMIT_WIDTH and ei.value.detail == w


def test_interval_boundary():
    assert S.build(cfg(strobe_interval_us=15))                 # width 10 + gap 5
    with pytest.raises(E.McuError) as ei:
        S.build(cfg(strobe_interval_us=14))
    assert (ei.value.code, ei.value.detail) == (E.LIMIT_TIMING, 5)
    assert S.build(cfg(pulses_per_exposure=1, strobe_interval_us=0))


@pytest.mark.parametrize("kw,field", [
    (dict(pulses_per_exposure=0), 2), (dict(pulses_per_exposure=33), 2),
    (dict(burst_exposures=0), 3), (dict(burst_exposures=9), 3),
    (dict(first_pulse_delay_us=0), 6), (dict(exposure_tail_us=0), 7),
    (dict(trigger_delay_us=1_000_001), 9), (dict(mode="nope"), 1),
])
def test_bad_values(kw, field):
    with pytest.raises(E.McuError) as ei:
        S.build(cfg(**kw))
    assert ei.value.code == E.BAD_VALUE and ei.value.detail == field


def test_exposure_length_boundary():
    base = dict(burst_exposures=1, pulses_per_exposure=1, first_pulse_delay_us=1)
    ok = S.build(cfg(exposure_tail_us=50_000 - 1 - 10, **base))
    assert ok.exposure_len_us == 50_000
    with pytest.raises(E.McuError) as ei:
        S.build(cfg(exposure_tail_us=50_000 - 10, **base))
    assert (ei.value.code, ei.value.detail) == (E.LIMIT_TIMING, 10)
    with pytest.raises(E.McuError):
        S.build(cfg(strobe_interval_us=2**32 - 1))


def test_duty_boundary_exact_2_percent():
    base = dict(mode="free_run", frame_period_us=10_000, strobe_interval_us=300)
    ok = S.build(cfg(pulses_per_exposure=20, **base))
    assert ok.duty_ppm_worst == 20_000
    with pytest.raises(E.McuError) as ei:
        S.build(cfg(pulses_per_exposure=21, **base))
    assert (ei.value.code, ei.value.detail) == (E.LIMIT_DUTY, 21_000)


def test_pps_boundary():
    base = dict(mode="free_run", strobe_width_us=5, pulses_per_exposure=1)
    assert S.build(cfg(frame_period_us=334, **base)).pps_worst == 2995
    with pytest.raises(E.McuError) as ei:
        S.build(cfg(frame_period_us=333, **base))
    assert (ei.value.code, ei.value.detail) == (E.LIMIT_PPS, 3004)


def test_big_burst_limits_and_ring():
    big = dict(burst_exposures=8, pulses_per_exposure=32, strobe_interval_us=100, strobe_width_us=20, frame_period_us=3320)
    with pytest.raises(E.McuError) as ei:
        S.build(cfg(**big), Options(lockout_us=0))
    assert ei.value.code == E.LIMIT_DUTY
    with pytest.raises(E.McuError):
        S.build(cfg(**big), Options(lockout_us=10_000_000))   # even a single burst is over 2 % of 100 ms
    ok = S.build(cfg(burst_exposures=8, pulses_per_exposure=32, strobe_interval_us=10, strobe_width_us=5,
                     frame_period_us=600))
    assert ok.n_words <= 1024 and ok.n_words & (ok.n_words - 1) == 0
    _, st, total = replay(ok)
    assert len(st) == 256 and total == ok.span_cycles


def test_lockout_raised_to_burst_span():
    s = S.build(StrobeConfig(), Options(lockout_us=0))
    assert s.lockout_eff_us == s.span_us + 10 and s.lockout_eff_us < 25_000


def test_polarity_not_in_words():
    a = S.build(StrobeConfig(), Options(cam_active_high=False, trig_rising=True))
    b = S.build(StrobeConfig(), Options(cam_active_high=True, trig_rising=False))
    assert a.words == b.words


def test_options_validation():
    for o in (Options(lockout_us=10_000_001), Options(vdiv_ratio=0), Options(status_period_ms=5)):
        with pytest.raises(E.McuError):
            S.build(StrobeConfig(), o)


def test_window_matches_bruteforce():
    import random
    rnd = random.Random(1)
    for _ in range(200):
        n = rnd.randint(1, 6)
        period = rnd.randint(50, 450)
        starts, p = [], rnd.randint(0, 9)
        for _ in range(n):
            starts.append(p)
            p += rnd.randint(1, max(1, (period - 1) // n))
        if starts[-1] >= period:
            continue
        window = rnd.randint(1, 3000)
        best = 0
        for s0 in range(period):
            cnt = sum(1 for k in range(3000 // period + 4) for st in starts if s0 <= k * period + st < s0 + window)
            best = max(best, cnt)
        assert S.max_pulses_in_window(starts, period, window) == best


def test_rail_mv():
    assert S.rail_mv(0, 3300, 4300) == 0
    assert 11950 < S.rail_mv(3463, 3300, 4300) < 12050
