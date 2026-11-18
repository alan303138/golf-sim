"""Wire protocol (see firmware/PROTOCOL.md): COBS + CRC-16 framing and message bodies.

Pure Python, no hardware. Byte-for-byte identical to firmware/lib/gs_proto.c (checked by
tests/test_crosscheck.py against the C implementation).
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

from gs_common.interfaces import StrobeConfig

from .errors import McuProtocolError

PROTO_MAJOR = 1
PROTO_MINOR = 0
MAX_BODY = 200
BOOTLOADER_MAGIC = 0xB00710AD

# message types
GET_INFO, SET_CONFIG, SET_OPTIONS, ARM, FIRE, GET_STATUS, GET_CONFIG, BOOTLOADER, CLEAR_FAULTS, GET_OPTIONS = (
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A)
RESULT, INFO, CONFIG, STATUS, TRIGGER, EVENT, OPTIONS = 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86

MODE_BURST, MODE_FREE_RUN = 0, 1
MODE_NAMES = {MODE_BURST: "burst", MODE_FREE_RUN: "free_run"}
SRC_NAMES = {0: "hardware", 1: "software"}
TRIG_FLAG_INHIBITED = 0x01
TRIG_FLAG_FREE_RUN = 0x02
STATUS_FLAG_STROBE_FORCED_LOW = 0x01

EVT_BOOT, EVT_FAULT, EVT_INTERLOCK, EVT_DISARMED = 1, 2, 3, 4


# ------------------------------------------------------------------ CRC / COBS
def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise ValueError("zero in COBS data")
        i += 1
        for _ in range(code - 1):
            if i >= n or data[i] == 0:
                raise ValueError("truncated COBS block")
            out.append(data[i])
            i += 1
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


# ---------------------------------------------------------------------- framing
@dataclass
class Frame:
    type: int
    tag: int
    body: bytes = b""


def encode_frame(type_: int, tag: int, body: bytes = b"") -> bytes:
    if len(body) > MAX_BODY:
        raise ValueError("body too long")
    if not (0 <= tag <= 255 and 0 <= type_ <= 255):
        raise ValueError("type/tag out of range")
    raw = bytes([type_, tag]) + body
    raw += struct.pack("<H", crc16(raw))
    return cobs_encode(raw) + b"\x00"


class Deframer:
    """Incremental decoder. feed() returns the valid frames found; bad frames are counted."""

    MAX_ENCODED = MAX_BODY + 4 + (MAX_BODY + 4) // 254 + 1

    def __init__(self) -> None:
        self._buf = bytearray()
        self._overflow = False
        self.bad_frames = 0

    def feed(self, data: bytes) -> list[Frame]:
        frames = []
        for b in data:
            if b != 0:
                if len(self._buf) < self.MAX_ENCODED:
                    self._buf.append(b)
                else:
                    self._overflow = True
                continue
            buf, ovf = bytes(self._buf), self._overflow
            self._buf.clear()
            self._overflow = False
            if not buf and not ovf:
                continue
            if ovf:
                self.bad_frames += 1
                continue
            try:
                raw = cobs_decode(buf)
            except ValueError:
                self.bad_frames += 1
                continue
            if len(raw) < 4 or struct.unpack("<H", raw[-2:])[0] != crc16(raw[:-2]):
                self.bad_frames += 1
                continue
            frames.append(Frame(raw[0], raw[1], raw[2:-2]))
        return frames


# ------------------------------------------------------------------ message bodies
@dataclass
class Options:
    """Hardware options carried by SET_OPTIONS (not part of gs_common.StrobeConfig)."""
    cam_active_high: bool = False      # False = CAM_TRIG active low (default)
    trig_rising: bool = True           # True = rising edge on TRIG_IN is the trigger
    status_period_ms: int = 1000       # 0 = no periodic STATUS
    lockout_us: int = 500_000          # TRIG_IN lockout after an accepted trigger
    vdiv_ratio: float = 4.3            # Vrail = Vadc * ratio


_OPTS = struct.Struct("<BBHII")


def pack_options(o: Options) -> bytes:
    return _OPTS.pack(int(o.cam_active_high), int(o.trig_rising), o.status_period_ms,
                      o.lockout_us, round(o.vdiv_ratio * 1000))


def unpack_options(b: bytes) -> Options:
    _check_len(b, _OPTS.size, "OPTIONS")
    a, r, p, l, v = _OPTS.unpack(b)
    return Options(bool(a), bool(r), p, l, v / 1000.0)


_CFG = struct.Struct("<BBBBIIIIII")


def pack_config(c: StrobeConfig) -> bytes:
    mode = {"burst": MODE_BURST, "free_run": MODE_FREE_RUN}.get(c.mode)
    if mode is None:
        raise ValueError(f"unknown mode {c.mode!r}")
    return _CFG.pack(mode, c.pulses_per_exposure, c.burst_exposures, 0, c.strobe_width_us,
                     c.strobe_interval_us, c.first_pulse_delay_us, c.exposure_tail_us,
                     c.frame_period_us, c.trigger_delay_us)


def unpack_config(b: bytes) -> StrobeConfig:
    _check_len(b, _CFG.size, "SET_CONFIG")
    m, n, e, _, w, i, f, t, fp, td = _CFG.unpack(b)
    return StrobeConfig(mode=MODE_NAMES.get(m, f"mode{m}"), strobe_width_us=w, strobe_interval_us=i,
                        pulses_per_exposure=n, first_pulse_delay_us=f, exposure_tail_us=t,
                        burst_exposures=e, frame_period_us=fp, trigger_delay_us=td)


@dataclass
class ConfigReport:
    config: StrobeConfig
    exposure_len_us: int
    span_us: int
    lockout_eff_us: int
    duty_ppm_worst: int
    pps_worst: int
    latency_ns: int


_REPORT = struct.Struct("<IIIIII")


def pack_config_report(r: ConfigReport) -> bytes:
    return pack_config(r.config) + _REPORT.pack(r.exposure_len_us, r.span_us, r.lockout_eff_us,
                                                r.duty_ppm_worst, r.pps_worst, r.latency_ns)


def unpack_config_report(b: bytes) -> ConfigReport:
    _check_len(b, _CFG.size + _REPORT.size, "CONFIG")
    return ConfigReport(unpack_config(b[:_CFG.size]), *_REPORT.unpack(b[_CFG.size:]))


@dataclass
class InfoMsg:
    proto_major: int
    proto_minor: int
    fw: tuple[int, int, int]
    hw_revision: int
    uid: bytes
    uptime_ms: int
    clk_hz: int
    max_width_us: int
    min_width_us: int
    max_duty_ppm: int
    duty_window_us: int
    max_pps: int
    max_pulses_per_exposure: int
    max_burst_exposures: int
    max_exposure_us: int

    @property
    def fw_version(self) -> str:
        return "{}.{}.{}".format(*self.fw)

    @property
    def serial(self) -> str:
        return self.uid.hex()


_INFO = struct.Struct("<6B8sIIIIIIIBBI")


def pack_info(m: InfoMsg) -> bytes:
    return _INFO.pack(m.proto_major, m.proto_minor, *m.fw, m.hw_revision, m.uid, m.uptime_ms, m.clk_hz,
                      m.max_width_us, m.min_width_us, m.max_duty_ppm, m.duty_window_us, m.max_pps,
                      m.max_pulses_per_exposure, m.max_burst_exposures, m.max_exposure_us)


def unpack_info(b: bytes) -> InfoMsg:
    _check_len(b, _INFO.size, "INFO")
    v = _INFO.unpack(b)
    return InfoMsg(v[0], v[1], (v[2], v[3], v[4]), v[5], v[6], *v[7:])


@dataclass
class StatusMsg:
    armed: bool
    mode: int
    interlock_ok: bool
    flags: int
    faults: int
    uptime_ms: int
    rail_mv: int
    trig_hw: int
    trig_sw: int
    trig_inhibited: int
    bursts_done: int
    pulses_total: int
    rx_bad_frames: int
    duty_ppm_now: int
    max_pulse_us_seen: int


_STATUS = struct.Struct("<4B11I")


def pack_status(m: StatusMsg) -> bytes:
    return _STATUS.pack(int(m.armed), m.mode, int(m.interlock_ok), m.flags, m.faults, m.uptime_ms,
                        m.rail_mv, m.trig_hw, m.trig_sw, m.trig_inhibited, m.bursts_done,
                        m.pulses_total, m.rx_bad_frames, m.duty_ppm_now, m.max_pulse_us_seen)


def unpack_status(b: bytes) -> StatusMsg:
    _check_len(b, _STATUS.size, "STATUS")
    v = _STATUS.unpack(b)
    return StatusMsg(bool(v[0]), v[1], bool(v[2]), *v[3:])


@dataclass
class TriggerMsg:
    seq: int
    t_us: int
    source: int
    flags: int
    exp_delta_us: list[int] = field(default_factory=list)
    pulse_offset_us: list[int] = field(default_factory=list)


_TRIG_HEAD = struct.Struct("<IQBBBB")


def pack_trigger(m: TriggerMsg) -> bytes:
    out = _TRIG_HEAD.pack(m.seq, m.t_us, m.source, m.flags, len(m.exp_delta_us), len(m.pulse_offset_us))
    out += struct.pack(f"<{len(m.exp_delta_us)}i", *m.exp_delta_us)
    out += struct.pack(f"<{len(m.pulse_offset_us)}H", *m.pulse_offset_us)
    return out


def unpack_trigger(b: bytes) -> TriggerMsg:
    if len(b) < _TRIG_HEAD.size:
        raise McuProtocolError("TRIGGER too short")
    seq, t, src, flags, ne, npul = _TRIG_HEAD.unpack_from(b)
    if len(b) != _TRIG_HEAD.size + 4 * ne + 2 * npul or ne > 8 or npul > 32:
        raise McuProtocolError("TRIGGER bad length")
    off = _TRIG_HEAD.size
    deltas = list(struct.unpack_from(f"<{ne}i", b, off))
    pulses = list(struct.unpack_from(f"<{npul}H", b, off + 4 * ne))
    return TriggerMsg(seq, t, src, flags, deltas, pulses)


_RESULT = struct.Struct("<BBI")
_EVENT = struct.Struct("<BI")


def pack_result(req: int, code: int, detail: int = 0) -> bytes:
    return _RESULT.pack(req, code, detail)


def unpack_result(b: bytes) -> tuple[int, int, int]:
    _check_len(b, _RESULT.size, "RESULT")
    return _RESULT.unpack(b)


def pack_event(code: int, value: int) -> bytes:
    return _EVENT.pack(code, value)


def unpack_event(b: bytes) -> tuple[int, int]:
    _check_len(b, _EVENT.size, "EVENT")
    return _EVENT.unpack(b)


def _check_len(b: bytes, n: int, what: str) -> None:
    if len(b) != n:
        raise McuProtocolError(f"{what}: expected {n} bytes, got {len(b)}")
