"""SerialMcuLink: McuLink over the Pico's USB CDC port."""
from __future__ import annotations

import logging
import os
import threading
from typing import Any, Callable

import serial
from serial.tools import list_ports

from gs_common.interfaces import McuInfo, McuLink, ShotTrigger, StrobeConfig

from . import errors as E
from . import protocol as P
from .convert import McuShotTrigger, info_from_msg, status_to_dict, trigger_from_msg

log = logging.getLogger("gs_mcu")

USB_VID = 0x2E8A            # Raspberry Pi
USB_PIDS = (0x000A,)        # pico-sdk CDC (RP2040); the firmware product string is "GolfSim-Timing"
PRODUCT_HINT = "GolfSim"
ENV_PORT = "GS_MCU_PORT"


def find_port(override: str | None = None) -> str:
    """Pico serial port: explicit override, $GS_MCU_PORT, else autodetect by USB VID/PID
    (product string "GolfSim" preferred over a generic Pico CDC)."""
    override = override or os.environ.get(ENV_PORT)
    if override:
        return override
    exact, generic = [], []
    for p in list_ports.comports():
        if p.vid != USB_VID or p.pid not in USB_PIDS:
            continue
        (exact if PRODUCT_HINT in (p.product or "") else generic).append(p.device)
    if exact:
        return sorted(exact)[0]
    if len(generic) == 1:
        return generic[0]
    if generic:
        raise E.McuNotFound(f"several Pico CDC ports found ({', '.join(sorted(generic))}); pass port= explicitly")
    raise E.McuNotFound("no Pico timing MCU found (USB 2e8a:000a); is it plugged in and not in BOOTSEL mode?")


class SerialMcuLink(McuLink):
    """Thread-safe blocking driver. Callbacks run on the reader thread."""

    def __init__(self, port: str | None = None, timeout: float = 1.0, baudrate: int = 115200,
                 serial_factory: Callable[..., Any] | None = None):
        self._port_arg = port
        self._timeout = timeout
        self._baud = baudrate
        self._factory = serial_factory or serial.Serial
        self._ser: Any = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._tx_lock = threading.Lock()
        self._lock = threading.Lock()
        self._tag = 0
        self._pending: dict[int, tuple[threading.Event, list]] = {}
        self._trigger_cbs: list[Callable[[ShotTrigger], None]] = []
        self._event_cbs: list[Callable[[str, int], None]] = []
        self._deframer = P.Deframer()
        self.options = P.Options()
        self.info_msg: P.InfoMsg | None = None
        self.last_status: dict | None = None
        self.last_config: P.ConfigReport | None = None
        self.port: str | None = None
        self._last_detail = 0
        self.strobe_inhibited_on_arm = False   # ARM succeeded but STROBE_EN is low

    # ---------------------------------------------------------------- lifecycle
    def open(self) -> None:
        if self._ser is not None:
            return
        self.port = find_port(self._port_arg)
        self._ser = self._factory(self.port, self._baud, timeout=0.05, write_timeout=self._timeout)
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, name="gs_mcu-reader", daemon=True)
        self._thread.start()
        try:
            msg = self._get_info_msg()
        except Exception:
            self.close()
            raise
        if msg.proto_major != P.PROTO_MAJOR:
            self.close()
            raise E.McuProtocolError(
                f"firmware protocol {msg.proto_major}.{msg.proto_minor} incompatible with host {P.PROTO_MAJOR}.{P.PROTO_MINOR}")
        self.info_msg = msg

    def close(self) -> None:
        self._stop.set()
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
        t, self._thread = self._thread, None
        if t is not None and t is not threading.current_thread():
            t.join(timeout=1.0)

    def __enter__(self) -> "SerialMcuLink":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------------ reader
    def _reader(self) -> None:
        while not self._stop.is_set():
            ser = self._ser
            if ser is None:
                break
            try:
                data = ser.read(max(1, getattr(ser, "in_waiting", 0) or 1))
            except Exception as exc:  # noqa: BLE001 - port vanished (e.g. reboot)
                if not self._stop.is_set():
                    log.warning("serial read failed: %s", exc)
                break
            if not data:
                continue
            for f in self._deframer.feed(data):
                self._dispatch(f)
        # wake any waiters so they fail fast
        with self._lock:
            for ev, slot in self._pending.values():
                if not slot:
                    slot.append(None)
                ev.set()

    def _dispatch(self, f: P.Frame) -> None:
        try:
            if f.type == P.TRIGGER:
                trig = trigger_from_msg(P.unpack_trigger(f.body))
                for cb in list(self._trigger_cbs):
                    self._call(cb, trig)
            elif f.type == P.EVENT:
                code, value = P.unpack_event(f.body)
                name = {P.EVT_BOOT: "boot", P.EVT_FAULT: "fault", P.EVT_INTERLOCK: "interlock",
                        P.EVT_DISARMED: "disarmed"}.get(code, f"event{code}")
                for cb in list(self._event_cbs):
                    self._call(cb, name, value)
            else:
                if f.type == P.STATUS:
                    self.last_status = status_to_dict(P.unpack_status(f.body), self._fw())
                with self._lock:
                    waiter = self._pending.get(f.tag) if f.tag else None
                if waiter is not None:
                    waiter[1].append(f)
                    waiter[0].set()
        except (E.McuProtocolError, ValueError) as exc:
            log.warning("bad message type 0x%02x: %s", f.type, exc)

    @staticmethod
    def _call(cb, *args) -> None:
        try:
            cb(*args)
        except Exception:  # noqa: BLE001
            log.exception("callback failed")

    def _fw(self) -> str:
        return self.info_msg.fw_version if self.info_msg else ""

    # ----------------------------------------------------------------- requests
    def _request(self, type_: int, body: bytes = b"", expect: tuple[int, ...] = (), name: str = "") -> P.Frame:
        if self._ser is None:
            raise E.McuTimeout("link not open")
        ev = threading.Event()
        slot: list = []
        with self._lock:
            self._tag = self._tag % 255 + 1
            tag = self._tag
            self._pending[tag] = (ev, slot)
        try:
            data = P.encode_frame(type_, tag, body)
            with self._tx_lock:
                self._ser.write(data)
            if not ev.wait(self._timeout) or not slot or slot[0] is None:
                raise E.McuTimeout(f"{name or hex(type_)}: no answer within {self._timeout} s")
            f: P.Frame = slot[0]
        finally:
            with self._lock:
                self._pending.pop(tag, None)
        if f.type == P.RESULT:
            _, code, detail = P.unpack_result(f.body)
            if code != E.OK:
                raise E.McuError(code, detail, name)
            if expect:
                raise E.McuProtocolError(f"{name}: got RESULT, expected 0x{expect[0]:02x}")
            self._last_detail = detail
            return f
        if expect and f.type not in expect:
            raise E.McuProtocolError(f"{name}: unexpected reply type 0x{f.type:02x}")
        return f

    def _get_info_msg(self) -> P.InfoMsg:
        return P.unpack_info(self._request(P.GET_INFO, expect=(P.INFO,), name="GET_INFO").body)

    # ------------------------------------------------------------ McuLink API
    def info(self) -> McuInfo:
        msg = self._get_info_msg()
        self.info_msg = msg
        return info_from_msg(msg)

    def configure(self, config: StrobeConfig) -> None:
        f = self._request(P.SET_CONFIG, P.pack_config(config), expect=(P.CONFIG,), name="SET_CONFIG")
        self.last_config = P.unpack_config_report(f.body)

    def set_options(self, options: P.Options) -> None:
        """Set hardware options (polarities, lockout, status period, divider ratio). Disarmed only."""
        self._request(P.SET_OPTIONS, P.pack_options(options), name="SET_OPTIONS")
        self.options = options

    def get_options(self) -> P.Options:
        f = self._request(P.GET_OPTIONS, expect=(P.OPTIONS,), name="GET_OPTIONS")
        self.options = P.unpack_options(f.body)
        return self.options

    def get_config(self) -> P.ConfigReport:
        f = self._request(P.GET_CONFIG, expect=(P.CONFIG,), name="GET_CONFIG")
        self.last_config = P.unpack_config_report(f.body)
        return self.last_config

    def arm(self, armed: bool) -> None:
        self._request(P.ARM, bytes([1 if armed else 0]), name="ARM")
        self.strobe_inhibited_on_arm = bool(armed and self._last_detail == 1)

    def fire(self) -> None:
        self._request(P.FIRE, name="FIRE")

    def on_trigger(self, callback: Callable[[ShotTrigger], None]) -> None:
        self._trigger_cbs.append(callback)

    def on_event(self, callback: Callable[[str, int], None]) -> None:
        """callback(name, value) for boot / fault / interlock / disarmed events."""
        self._event_cbs.append(callback)

    def status(self) -> dict[str, Any]:
        f = self._request(P.GET_STATUS, expect=(P.STATUS,), name="GET_STATUS")
        d = status_to_dict(P.unpack_status(f.body), self._fw())
        self.last_status = d
        return d

    def clear_faults(self) -> None:
        self._request(P.CLEAR_FAULTS, name="CLEAR_FAULTS")

    def reboot_to_bootloader(self) -> None:
        """Reboot into the USB mass-storage bootloader (BOOTSEL) for a UF2 update; closes the link."""
        try:
            self._request(P.BOOTLOADER, P.BOOTLOADER_MAGIC.to_bytes(4, "little"), name="BOOTLOADER")
        finally:
            self.close()
