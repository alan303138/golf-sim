import dataclasses

import pytest

from gs_common.interfaces import McuLink, ShotTrigger, StrobeConfig
from gs_mcu import errors as E
from gs_mcu.mock import ManualClock, MockMcuLink
from gs_mcu.protocol import Options


@pytest.fixture
def mock():
    clock = ManualClock()
    m = MockMcuLink(clock=clock)
    m.clock = clock
    m.open()
    yield m
    m.close()


def test_is_mculink_and_info(mock):
    assert isinstance(mock, McuLink)
    i = mock.info()
    assert i.fw_version == "1.0.0" and i.hw_revision == 1 and i.serial


def test_configure_arm_fire_callbacks(mock):
    got = []
    mock.on_trigger(got.append)
    mock.configure(StrobeConfig(trigger_delay_us=100))
    with pytest.raises(E.McuError) as ei:
        mock.fire()
    assert ei.value.code == E.NOT_ARMED
    mock.arm(True)
    t0 = mock.clock.now_us
    mock.fire()
    assert len(got) == 1 and isinstance(got[0], ShotTrigger)
    t = got[0]
    assert t.seq == 1 and t.source == "software" and t.t_us == t0
    assert t.exposure_start_us == [t0 + 100, t0 + 100 + 16700]
    assert t.pulse_offsets_us == [50, 750, 1450, 2150, 2850]
    assert not t.strobe_inhibited


def test_lockout_applies_to_fire_and_hardware(mock):
    mock.configure(StrobeConfig())
    mock.arm(True)
    mock.fire()
    mock.clock.advance(1000)
    with pytest.raises(E.McuError) as ei:
        mock.fire()
    assert ei.value.code == E.LOCKOUT and ei.value.detail == 500_000 - 1000
    assert mock.simulate_trigger() is None            # PIO ignores hardware edges during the lockout
    mock.clock.advance(500_000)
    t = mock.simulate_trigger()
    assert t is not None and t.source == "hardware" and t.seq == 2


def test_config_rejected_limits_and_state(mock):
    with pytest.raises(E.McuError) as ei:
        mock.configure(StrobeConfig(strobe_width_us=21))
    assert ei.value.code == E.LIMIT_WIDTH
    assert mock.config.strobe_width_us == 10            # unchanged
    mock.arm(True)
    with pytest.raises(E.McuError) as ei:
        mock.configure(StrobeConfig())
    assert ei.value.code == E.BUSY_ARMED
    with pytest.raises(E.McuError):
        mock.set_options(Options())
    mock.arm(False)
    mock.set_options(Options(lockout_us=0))
    assert mock.schedule.lockout_eff_us == mock.schedule.span_us + 10


def test_free_run_report(mock):
    mock.configure(StrobeConfig(mode="free_run"))
    mock.arm(True)
    epoch = mock.clock.now_us
    mock.clock.advance(3 * 16700 + 1200)
    t = mock.simulate_trigger()
    assert t.free_run
    assert t.exposure_start_us == [epoch + 3 * 16700, epoch + 4 * 16700]
    assert t.t_us - t.exposure_start_us[0] == 1200


def test_faults_disarm_and_latch(mock):
    events = []
    mock.on_event(lambda n, v: events.append((n, v)))
    mock.configure(StrobeConfig())
    mock.arm(True)
    mock.simulate_fault(0x01)                           # strobe over-width watchdog
    assert not mock.status()["armed"] and "STROBE_OVERWIDTH" in mock.status()["faults"]
    assert ("disarmed", 1) in events and ("fault", 1) in events
    with pytest.raises(E.McuError) as ei:
        mock.arm(True)
    assert ei.value.code == E.FAULT_LATCHED
    assert mock.simulate_trigger() is None
    mock.clear_faults()
    mock.arm(True)
    assert mock.status()["armed"]


def test_interlock_inhibits_strobe(mock):
    got = []
    mock.on_trigger(got.append)
    mock.configure(StrobeConfig())
    mock.simulate_interlock(False)
    mock.arm(True)
    assert mock.strobe_inhibited_on_arm
    t = mock.simulate_trigger()
    assert t.strobe_inhibited and t.pulse_offsets_us == []
    st = mock.status()
    assert not st["interlock_ok"] and st["strobe_forced_low"] and st["trigger_inhibited"] == 1
    assert "INTERLOCK_TRIPPED" in st["faults"]
    assert st["armed"]                                  # camera keeps running, strobe inhibited


def test_status_fields(mock):
    mock.set_rail_mv(11870)
    st = mock.status()
    assert st["rail_v"] == pytest.approx(11.87) and st["mode"] == "burst" and st["fw_version"] == "1.0.0"
    assert set(["armed", "faults", "interlock_ok", "rail_mv", "trigger_hw", "trigger_sw"]) <= set(st)


def test_default_real_clock_and_closed_link():
    m = MockMcuLink()
    with pytest.raises(E.McuTimeout):
        m.status()
    with m:
        m.configure(dataclasses.replace(StrobeConfig(), burst_exposures=1))
        m.arm(True)
        m.fire()
        assert m.triggers[0].exposure_start_us == [m.triggers[0].t_us]
