import struct

import pytest

from gs_common.interfaces import StrobeConfig
from gs_mcu import protocol as P
from gs_mcu.errors import McuProtocolError


def test_crc_check_value():
    assert P.crc16(b"123456789") == 0x29B1
    assert P.crc16(b"") == 0xFFFF


@pytest.mark.parametrize("raw,enc", [
    (b"\x00", b"\x01\x01"),
    (b"\x00\x00", b"\x01\x01\x01"),
    (b"\x11\x22\x00\x33", b"\x03\x11\x22\x02\x33"),
    (b"\x11\x22\x33\x44", b"\x05\x11\x22\x33\x44"),
    (b"", b"\x01"),
])
def test_cobs_vectors(raw, enc):
    assert P.cobs_encode(raw) == enc
    assert P.cobs_decode(enc) == raw


def test_cobs_long_runs():
    for n in (253, 254, 255, 256, 508, 1000):
        raw = bytes((i % 255) + 1 for i in range(n))
        enc = P.cobs_encode(raw)
        assert 0 not in enc and P.cobs_decode(enc) == raw
    raw = bytes(range(255))
    assert P.cobs_decode(P.cobs_encode(raw)) == raw


def test_cobs_errors():
    for bad in (b"\x05\x11\x22", b"\x02\x00"):
        with pytest.raises(ValueError):
            P.cobs_decode(bad)


def test_frame_roundtrip_and_resync():
    d = P.Deframer()
    f1 = P.encode_frame(P.ARM, 7, b"\x01")
    f2 = P.encode_frame(P.GET_INFO, 8)
    assert f1.endswith(b"\x00") and f1.count(b"\x00") == 1
    got = d.feed(b"\x00\x00" + f1 + f2)
    assert [(f.type, f.tag, f.body) for f in got] == [(P.ARM, 7, b"\x01"), (P.GET_INFO, 8, b"")]
    # byte at a time
    got = []
    for b in f1:
        got += d.feed(bytes([b]))
    assert len(got) == 1
    # corrupted frames are dropped and counted, the next good one still decodes
    bad = bytearray(f1)
    bad[2] ^= 0x10
    assert d.feed(bytes(bad)) == []
    assert d.bad_frames == 1
    assert len(d.feed(f2)) == 1
    d.feed(b"\x55" * 1000 + b"\x00")
    assert d.bad_frames == 2
    assert len(d.feed(f1)) == 1


def test_all_single_bit_flips_rejected():
    frame = P.encode_frame(P.FIRE, 3, bytes([9, 8, 7]))
    d = P.Deframer()
    for i in range(len(frame) - 1):
        for bit in range(8):
            c = bytearray(frame)
            c[i] ^= 1 << bit
            if c[i] == 0:
                continue
            assert d.feed(bytes(c)) == []


def test_max_body():
    body = bytes(range(200))
    f = P.encode_frame(0x42, 1, body)
    assert P.Deframer().feed(f)[0].body == body
    with pytest.raises(ValueError):
        P.encode_frame(1, 1, bytes(201))


def test_config_roundtrip_and_layout():
    c = StrobeConfig(mode="free_run", strobe_width_us=12, strobe_interval_us=345, pulses_per_exposure=7,
                     first_pulse_delay_us=67, exposure_tail_us=89, burst_exposures=3, frame_period_us=12345,
                     trigger_delay_us=678901)
    b = P.pack_config(c)
    assert len(b) == 28
    assert b[:4] == bytes([1, 7, 3, 0])
    assert struct.unpack("<6I", b[4:]) == (12, 345, 67, 89, 12345, 678901)
    assert P.unpack_config(b) == c
    with pytest.raises(McuProtocolError):
        P.unpack_config(b[:-1])
    with pytest.raises(ValueError):
        P.pack_config(StrobeConfig(mode="bogus"))


def test_options_roundtrip():
    o = P.Options(cam_active_high=True, trig_rising=False, status_period_ms=250, lockout_us=123456, vdiv_ratio=11.0)
    b = P.pack_options(o)
    assert len(b) == 12 and b[0] == 1 and b[1] == 0
    assert P.unpack_options(b) == o


def test_trigger_roundtrip_negative_delta():
    m = P.TriggerMsg(seq=77, t_us=0x1122334455667788, source=1, flags=2, exp_delta_us=[-1200, 15500],
                     pulse_offset_us=[50, 750, 1450])
    b = P.pack_trigger(m)
    assert len(b) == 16 + 8 + 6
    assert P.unpack_trigger(b) == m
    with pytest.raises(McuProtocolError):
        P.unpack_trigger(b + b"\x00")


def test_status_info_sizes():
    s = P.StatusMsg(True, 1, False, 1, 0x81, 99, 11998, 1, 2, 3, 4, 5, 6, 7, 8)
    assert len(P.pack_status(s)) == 48 and P.unpack_status(P.pack_status(s)) == s
    i = P.InfoMsg(1, 0, (1, 2, 3), 1, bytes(range(8)), 5, 125_000_000, 20, 5, 20000, 100000, 3000, 32, 8, 50000)
    assert len(P.pack_info(i)) == 48 and P.unpack_info(P.pack_info(i)) == i
    assert i.fw_version == "1.2.3" and i.serial == "0001020304050607"
