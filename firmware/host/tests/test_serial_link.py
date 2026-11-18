import threading

import pytest

from gs_common.interfaces import McuLink, StrobeConfig
from gs_mcu import errors as E
from gs_mcu import protocol as P
from gs_mcu import serial_link as SL
from gs_mcu.serial_link import SerialMcuLink
from emulator import FakeSerial


def make(**kw):
    holder = {}

    def factory(*a, **k):
        holder["s"] = FakeSerial(*a, **kw, **k)
        return holder["s"]

    link = SerialMcuLink(port="FAKE", timeout=0.5, serial_factory=factory)
    return link, holder


@pytest.fixture
def link():
    l, h = make()
    l.open()
    l.fake = h["s"]
    yield l
    l.close()


def test_open_info(link):
    assert isinstance(link, McuLink)
    i = link.info()
    assert (i.fw_version, i.hw_revision, i.serial, i.uptime_ms) == ("1.0.0", 1, "0001020304050607", 5000)
    assert link.info_msg.max_width_us == 20


def test_configure_arm_fire_trigger_callback(link):
    got = threading.Event()
    seen = []
    link.on_trigger(lambda t: (seen.append(t), got.set()))
    link.set_options(P.Options(cam_active_high=False, trig_rising=True, lockout_us=200_000))
    link.configure(StrobeConfig(trigger_delay_us=250))
    assert link.last_config.exposure_len_us == 2910
    link.arm(True)
    link.fire()
    assert got.wait(1.0)
    t = seen[0]
    assert t.source == "software" and t.seq == 1
    assert t.exposure_start_us == [t.t_us + 250, t.t_us + 250 + 16700]
    assert t.pulse_offsets_us == [50, 750, 1450, 2150, 2850]
    assert link.status()["trigger_sw"] == 1
    assert link.get_options().lockout_us == 200_000
    assert link.get_config().config.trigger_delay_us == 250


def test_rejections_raise_mcu_error(link):
    with pytest.raises(E.McuError) as ei:
        link.configure(StrobeConfig(strobe_width_us=25))
    assert ei.value.code == E.LIMIT_WIDTH and "LIMIT_WIDTH" in str(ei.value) and "25" in str(ei.value)
    with pytest.raises(E.McuError) as ei:
        link.fire()
    assert ei.value.code == E.NOT_ARMED
    link.arm(True)
    with pytest.raises(E.McuError) as ei:
        link.configure(StrobeConfig())
    assert ei.value.code == E.BUSY_ARMED


def test_events_and_faults(link):
    events = []
    ev = threading.Event()
    link.on_event(lambda n, v: (events.append((n, v)), ev.set()))
    link.arm(True)
    link.fake.dev.simulate_fault(0x01)
    assert ev.wait(1.0)
    assert ("disarmed", 1) in events
    assert link.status()["faults"] == ["STROBE_OVERWIDTH"]
    link.clear_faults()
    assert link.status()["faults"] == []


def test_timeout_when_silent():
    l, h = make(drop_replies=True)
    l._timeout = 0.2
    with pytest.raises(E.McuTimeout):
        l.open()
    assert l._ser is None


def test_incompatible_protocol_refused():
    l, h = make(proto_major=2)
    with pytest.raises(E.McuProtocolError):
        l.open()


def test_bootloader_sends_magic_and_closes(link):
    link.reboot_to_bootloader()
    assert link._ser is None
    f = [x for x in link.fake.written_frames if x.type == P.BOOTLOADER][0]
    assert int.from_bytes(f.body, "little") == P.BOOTLOADER_MAGIC


def test_callback_exception_does_not_kill_reader(link):
    ok = threading.Event()
    link.on_trigger(lambda t: 1 / 0)
    link.on_trigger(lambda t: ok.set())
    link.arm(True)
    link.fire()
    assert ok.wait(1.0)
    assert link.status()["armed"]


def test_find_port_autodetect(monkeypatch):
    class P_:
        def __init__(self, dev, vid, pid, product):
            self.device, self.vid, self.pid, self.product = dev, vid, pid, product

    monkeypatch.delenv(SL.ENV_PORT, raising=False)
    ports = [P_("/dev/ttyACM9", 0x1234, 0x000A, "x"), P_("/dev/ttyACM1", 0x2E8A, 0x000A, "Pico"),
             P_("/dev/ttyACM0", 0x2E8A, 0x000A, "GolfSim-Timing")]
    monkeypatch.setattr(SL.list_ports, "comports", lambda: ports)
    assert SL.find_port() == "/dev/ttyACM0"
    assert SL.find_port("/dev/override") == "/dev/override"
    monkeypatch.setenv(SL.ENV_PORT, "/dev/fromenv")
    assert SL.find_port() == "/dev/fromenv"
    monkeypatch.delenv(SL.ENV_PORT)
    monkeypatch.setattr(SL.list_ports, "comports", lambda: ports[:2])
    assert SL.find_port() == "/dev/ttyACM1"                  # single generic Pico CDC
    monkeypatch.setattr(SL.list_ports, "comports", lambda: [ports[0]])
    with pytest.raises(E.McuNotFound):
        SL.find_port()
    monkeypatch.setattr(SL.list_ports, "comports", lambda: [P_("/dev/a", 0x2E8A, 0xA, "Pico"), P_("/dev/b", 0x2E8A, 0xA, "Pico")])
    with pytest.raises(E.McuNotFound):
        SL.find_port()
