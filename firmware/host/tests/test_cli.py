import json

from gs_mcu.cli import main
from gs_mcu.mock import MockMcuLink


def run(capsys, *argv, factory=None):
    rc = main(list(argv), link_factory=factory)
    out = capsys.readouterr()
    return rc, out.out, out.err


def test_info_status_mock(capsys):
    rc, out, _ = run(capsys, "--mock", "info")
    assert rc == 0 and json.loads(out)["fw_version"] == "1.0.0"
    rc, out, _ = run(capsys, "--mock", "status")
    assert rc == 0 and json.loads(out)["armed"] is False


def test_configure_ok_and_rejected(capsys):
    rc, out, _ = run(capsys, "--mock", "configure", "--width-us", "20", "--cam-active", "high", "--trig-edge", "falling")
    assert rc == 0 and json.loads(out)["accepted"] is True
    rc, out, err = run(capsys, "--mock", "configure", "--width-us", "21")
    assert rc == 1 and "LIMIT_WIDTH" in err and out == ""
    rc, _, err = run(capsys, "--mock", "configure", "--pulses", "40")
    assert rc == 1 and "BAD_VALUE" in err and "pulses_per_exposure" in err


def test_arm_fire_flow_with_shared_mock(capsys):
    m = MockMcuLink()
    f = lambda a: m  # noqa: E731
    m.close = lambda: None  # keep state across CLI invocations
    assert run(capsys, "--mock", "fire", factory=f)[0] == 1          # not armed
    assert run(capsys, "--mock", "arm", factory=f)[0] == 0
    rc, out, _ = run(capsys, "--mock", "fire", factory=f)
    assert rc == 0 and out.strip() == "fired"
    assert len(m.triggers) == 1
    rc, out, _ = run(capsys, "--mock", "arm", "--off", factory=f)
    assert out.strip() == "disarmed"


def test_monitor_mock_prints_trigger(capsys):
    rc, out, _ = run(capsys, "--mock", "monitor", "--duration", "0.3", "--fire-every", "0.1")
    assert rc == 0 and "TRIGGER" in out and "STATUS" in out
    line = [l for l in out.splitlines() if l.startswith("TRIGGER")][0]
    d = json.loads(line.split(" ", 1)[1])
    assert d["pulse_offsets_us"] == [50, 750, 1450, 2150, 2850]


def test_bootloader_needs_confirmation(capsys):
    rc, _, err = run(capsys, "--mock", "bootloader")
    assert rc == 1 and "--yes" in err
    rc, out, _ = run(capsys, "--mock", "bootloader", "--yes")
    assert rc == 0 and "bootloader" in out


def test_no_device(capsys, monkeypatch):
    import gs_mcu.serial_link as SL
    monkeypatch.delenv(SL.ENV_PORT, raising=False)
    monkeypatch.setattr(SL.list_ports, "comports", lambda: [])
    rc, _, err = run(capsys, "info")
    assert rc == 2 and "no Pico" in err
    rc, _, err = run(capsys, "ports")
    assert rc == 2
