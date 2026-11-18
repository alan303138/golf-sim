"""A fake serial port with a firmware emulator behind it (uses MockMcuLink for the device logic)."""
import queue
import threading

from gs_common.interfaces import StrobeConfig
from gs_mcu import errors as E
from gs_mcu import protocol as P
from gs_mcu import schedule as S
from gs_mcu.mock import MockMcuLink


class FakeSerial:
    in_waiting = 0

    def __init__(self, *a, drop_replies=False, proto_major=P.PROTO_MAJOR, **k):
        self.dev = MockMcuLink()
        self.dev.open()
        self.dev.on_trigger(self._on_trigger)
        self.dev.on_event(lambda n, v: self._send(P.EVENT, 0, P.pack_event(
            {"boot": 1, "fault": 2, "interlock": 3, "disarmed": 4}[n], v)))
        self.rx = queue.Queue()
        self.deframer = P.Deframer()
        self.closed = False
        self.drop = drop_replies
        self.proto_major = proto_major
        self.written_frames = []

    # --- pyserial API
    def write(self, data):
        for f in self.deframer.feed(data):
            self.written_frames.append(f)
            if not self.drop:
                self._handle(f)
        return len(data)

    def read(self, n=1):
        try:
            return self.rx.get(timeout=0.02)
        except queue.Empty:
            return b""

    def close(self):
        self.closed = True

    # --- emulator
    def _send(self, type_, tag, body=b""):
        self.rx.put(P.encode_frame(type_, tag, body))

    def _on_trigger(self, t):
        deltas = [s - t.t_us for s in t.exposure_start_us]
        flags = (1 if t.strobe_inhibited else 0) | (2 if t.free_run else 0)
        self._send(P.TRIGGER, 0, P.pack_trigger(P.TriggerMsg(
            t.seq, t.t_us, 1 if t.source == "software" else 0, flags, deltas, t.pulse_offsets_us)))

    def _result(self, f, code=0, detail=0):
        self._send(P.RESULT, f.tag, P.pack_result(f.type, code, detail))

    def _handle(self, f):
        d = self.dev
        try:
            if f.type == P.GET_INFO:
                self._send(P.INFO, f.tag, P.pack_info(P.InfoMsg(
                    self.proto_major, 0, (1, 0, 0), 1, bytes(range(8)), 5000, S.CLK_HZ, 20, 5, 20000, 100000, 3000, 32, 8, 50000)))
            elif f.type == P.SET_CONFIG:
                d.configure(P.unpack_config(f.body))
                self._send(P.CONFIG, f.tag, P.pack_config_report(d.get_config()))
            elif f.type == P.SET_OPTIONS:
                d.set_options(P.unpack_options(f.body))
                self._result(f)
            elif f.type == P.GET_OPTIONS:
                self._send(P.OPTIONS, f.tag, P.pack_options(d.get_options()))
            elif f.type == P.GET_CONFIG:
                self._send(P.CONFIG, f.tag, P.pack_config_report(d.get_config()))
            elif f.type == P.ARM:
                d.arm(bool(f.body[0]))
                self._result(f, 0, 1 if d.strobe_inhibited_on_arm and f.body[0] else 0)
            elif f.type == P.FIRE:
                d.fire()
                self._result(f)
            elif f.type == P.GET_STATUS:
                st = d.status()
                self._send(P.STATUS, f.tag, P.pack_status(P.StatusMsg(
                    st["armed"], P.MODE_BURST, st["interlock_ok"], 0, st["faults_mask"], 7, st["rail_mv"],
                    st["trigger_hw"], st["trigger_sw"], 0, 0, 0, 0, 0, 0)))
            elif f.type == P.CLEAR_FAULTS:
                d.clear_faults()
                self._result(f)
            elif f.type == P.BOOTLOADER:
                magic = int.from_bytes(f.body, "little")
                self._result(f, 0 if magic == P.BOOTLOADER_MAGIC else E.BAD_MAGIC)
            else:
                self._result(f, E.UNKNOWN_CMD, f.type)
        except E.McuError as exc:
            self._result(f, exc.code, exc.detail)
