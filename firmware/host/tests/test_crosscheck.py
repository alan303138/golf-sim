"""Byte-for-byte cross-check of the Python protocol/schedule against the C implementation."""
import dataclasses
import random
import subprocess

import pytest

from gs_common.interfaces import StrobeConfig
from gs_mcu import errors as E
from gs_mcu import protocol as P
from gs_mcu import schedule as S


def run_tool(tool, mode, text=""):
    return subprocess.run([tool, mode], input=text, capture_output=True, text=True, check=True).stdout.splitlines()


@pytest.fixture(scope="module")
def c_frames(vectors_tool):
    return dict(line.split(" ") for line in run_tool(vectors_tool, "frames"))


def py_vectors():
    cfg = StrobeConfig(mode="free_run", strobe_width_us=12, strobe_interval_us=345, pulses_per_exposure=7,
                       first_pulse_delay_us=67, exposure_tail_us=89, burst_exposures=3, frame_period_us=12345,
                       trigger_delay_us=678901)
    opts = P.Options(cam_active_high=True, trig_rising=False, status_period_ms=250, lockout_us=123456, vdiv_ratio=11.0)
    v = {
        "set_config": P.encode_frame(P.SET_CONFIG, 17, P.pack_config(cfg)),
        "set_options": P.encode_frame(P.SET_OPTIONS, 18, P.pack_options(opts)),
        "arm": P.encode_frame(P.ARM, 1, b"\x01"),
        "fire": P.encode_frame(P.FIRE, 255),
        "get_info": P.encode_frame(P.GET_INFO, 3),
        "bootloader": P.encode_frame(P.BOOTLOADER, 4, P.BOOTLOADER_MAGIC.to_bytes(4, "little")),
        "info": P.encode_frame(P.INFO, 3, P.pack_info(P.InfoMsg(
            1, 0, (1, 2, 3), 1, bytes([1, 2, 0, 4, 5, 0, 7, 8]), 123456, 125_000_000, 20, 5, 20000, 100000, 3000, 32, 8, 50000))),
        "status": P.encode_frame(P.STATUS, 0, P.pack_status(P.StatusMsg(
            True, 1, False, 1, 0x81, 99999, 11998, 1, 2, 3, 4, 5, 6, 7, 8))),
        "trigger": P.encode_frame(P.TRIGGER, 0, P.pack_trigger(P.TriggerMsg(
            77, 0x1122334455667788, 1, 2, [-1200, 15500], [50, 750, 1450]))),
        "result": P.encode_frame(P.RESULT, 17, P.pack_result(P.SET_CONFIG, E.LIMIT_DUTY, 21000)),
        "event": P.encode_frame(P.EVENT, 0, P.pack_event(P.EVT_FAULT, 1)),
        "config": P.encode_frame(P.CONFIG, 17, P.pack_config_report(P.ConfigReport(cfg, 1, 2, 3, 4, 5, 6))),
        "options": P.encode_frame(P.OPTIONS, 9, P.pack_options(opts)),
    }
    return v


def test_c_and_python_frames_identical(c_frames):
    py = py_vectors()
    assert set(py) == set(c_frames)
    for name, frame in py.items():
        assert frame.hex() == c_frames[name], name


def test_c_decodes_python_frames_and_roundtrips(vectors_tool):
    py = py_vectors()
    out = run_tool(vectors_tool, "decode", "\n".join(f.hex() for f in py.values()) + "\n")
    assert len(out) == len(py)
    for (name, frame), line in zip(py.items(), out):
        typ, tag, body, rt = line.split(" ")
        f = P.Deframer().feed(frame)[0]
        assert (int(typ), int(tag), body) == (f.type, f.tag, f.body.hex()), name
        assert rt == "1", f"C unpack/pack round trip differs for {name}"


def test_c_rejects_corrupt_frames(vectors_tool):
    frame = bytearray(P.encode_frame(P.ARM, 1, b"\x01"))
    frame[1] ^= 0x04
    assert run_tool(vectors_tool, "decode", frame.hex() + "\n") == ["BAD"]


def test_random_frames_agree(vectors_tool):
    rnd = random.Random(7)
    frames = []
    for _ in range(300):
        body = bytes(rnd.choice([0, rnd.randrange(256)]) for _ in range(rnd.randint(0, 200)))
        frames.append(P.encode_frame(0x40 + rnd.randrange(8), rnd.randrange(256), body))   # unknown types: no round trip
    out = run_tool(vectors_tool, "decode", "\n".join(f.hex() for f in frames) + "\n")
    for frame, line in zip(frames, out):
        f = P.Deframer().feed(frame)[0]
        assert line.split(" ")[:3] == [str(f.type), str(f.tag), f.body.hex()]


def sched_line(c: StrobeConfig, o: P.Options) -> str:
    mode = 1 if c.mode == "free_run" else 0
    return (f"{mode} {c.pulses_per_exposure} {c.burst_exposures} {c.strobe_width_us} {c.strobe_interval_us} "
            f"{c.first_pulse_delay_us} {c.exposure_tail_us} {c.frame_period_us} {c.trigger_delay_us} "
            f"{int(o.cam_active_high)} {int(o.trig_rising)} {o.lockout_us}")


def py_sched_line(c: StrobeConfig, o: P.Options) -> str:
    try:
        s = S.build(c, o)
    except E.McuError as e:
        return f"ERR {e.code} {e.detail}"
    words = "".join(f"{w:08x}" for w in s.words)
    return (f"OK {s.n_words} {s.n_real} {s.exposure_len_us} {s.span_us} {s.lockout_eff_us} {s.duty_ppm_worst} "
            f"{s.pps_worst} {s.span_cycles} | {' '.join(map(str, s.exposure_start_rel_us))} | "
            f"{' '.join(map(str, s.pulse_offsets_us))} | {words}")


def random_cfg(rnd):
    return StrobeConfig(
        mode=rnd.choice(["burst", "free_run"]),
        strobe_width_us=rnd.choice([4, 5, 10, 20, 21, rnd.randint(1, 30)]),
        strobe_interval_us=rnd.choice([0, 14, 15, 25, 700, rnd.randint(1, 3000)]),
        pulses_per_exposure=rnd.choice([0, 1, 5, 32, 33, rnd.randint(1, 32)]),
        first_pulse_delay_us=rnd.choice([0, 1, 50, rnd.randint(1, 500)]),
        exposure_tail_us=rnd.choice([0, 1, 50, rnd.randint(1, 500)]),
        burst_exposures=rnd.choice([0, 1, 2, 8, 9, rnd.randint(1, 8)]),
        frame_period_us=rnd.choice([0, 3010, 3009, 16700, 1_000_001, rnd.randint(100, 40000)]),
        trigger_delay_us=rnd.choice([0, 1234, 1_000_000, 1_000_001, rnd.randint(0, 5000)]),
    )


def test_schedules_agree_with_c(vectors_tool):
    rnd = random.Random(2024)
    cases = [(StrobeConfig(), P.Options()),
             (dataclasses.replace(StrobeConfig(), mode="free_run"), P.Options()),
             (dataclasses.replace(StrobeConfig(), strobe_width_us=20), P.Options(lockout_us=0)),
             (dataclasses.replace(StrobeConfig(), mode="free_run", frame_period_us=10000, strobe_interval_us=300,
                                  pulses_per_exposure=20), P.Options()),
             (dataclasses.replace(StrobeConfig(), mode="free_run", frame_period_us=10000, strobe_interval_us=300,
                                  pulses_per_exposure=21), P.Options())]
    for _ in range(600):
        cases.append((random_cfg(rnd), P.Options(cam_active_high=rnd.random() < .5, trig_rising=rnd.random() < .5,
                                                 lockout_us=rnd.choice([0, 1000, 500_000, 10_000_000, 10_000_001]))))
    for _ in range(500):   # mostly valid configs: exercise the word generation and the limit maths
        w = rnd.randint(5, 20)
        n = rnd.randint(1, 32)
        iv = rnd.randint(w + 5, 1500)
        first, tail = rnd.randint(1, 200), rnd.randint(1, 200)
        exp = first + (n - 1) * iv + w + tail
        cases.append((StrobeConfig(mode=rnd.choice(["burst", "free_run"]), strobe_width_us=w, strobe_interval_us=iv,
                                   pulses_per_exposure=n, first_pulse_delay_us=first, exposure_tail_us=tail,
                                   burst_exposures=rnd.randint(1, 8),
                                   frame_period_us=exp + rnd.choice([99, 100, 500, 13000]),
                                   trigger_delay_us=rnd.choice([0, rnd.randint(0, 5000)])),
                      P.Options(lockout_us=rnd.choice([0, 1000, 500_000]))))
    c_out = run_tool(vectors_tool, "sched", "\n".join(sched_line(c, o) for c, o in cases) + "\n")
    assert len(c_out) == len(cases)
    n_ok = 0
    for (c, o), line in zip(cases, c_out):
        assert py_sched_line(c, o) == line, (c, o)
        n_ok += line.startswith("OK")
    assert 50 < n_ok < len(cases) - 50     # the random mix really exercises both accepted and rejected configs
