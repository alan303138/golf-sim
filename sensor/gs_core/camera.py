"""Camera abstraction for the externally triggered IMX296 ball cameras.

A camera delivers raw mono frames through callbacks on its own thread. In
external-trigger mode one frame is produced per trigger pulse from the Pico,
and the exposure equals the trigger pulse width (+~14 us on IMX296), so the
host never sets exposure; it only fixes the analogue gain and turns every
automatic algorithm off.

Frames are matched to MCU exposures by timestamp order (see
:class:`gs_core.pipeline.FrameMatcher`); ``Frame.timestamp_ns`` is the sensor
timestamp (libcamera ``SensorTimestamp``, CLOCK_MONOTONIC/BOOTTIME domain on
the Pi) and only has to be consistent within one camera.
"""
from __future__ import annotations

import logging
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

log = logging.getLogger(__name__)

IMX296_SIZE = (1456, 1088)


@dataclass
class CameraConfig:
    size: tuple[int, int] = IMX296_SIZE   # full sensor; a crop is a (x, y, w, h) below
    crop: tuple[int, int, int, int] | None = None
    external_trigger: bool = True
    analogue_gain: float = 4.0            # fixed; chosen so a ball image sits at ~30-50 % of full scale
    exposure_us: int = 5000               # only used without external trigger
    buffer_count: int = 6


@dataclass
class Frame:
    camera: str
    seq: int                              # camera-side sequence number
    timestamp_ns: int                     # sensor timestamp (start of exposure where available)
    image: np.ndarray                     # (H, W) uint16, raw counts incl. black level
    black_level: float = 0.0
    full_scale: int = 1023
    meta: dict[str, Any] = field(default_factory=dict)
    host_time: float = field(default_factory=time.monotonic)


FrameCallback = Callable[[Frame], None]


class Camera(ABC):
    """Abstract camera; implementations call ``_emit`` from their own thread."""

    def __init__(self, name: str) -> None:
        self.name = name
        self._callbacks: list[FrameCallback] = []
        self._cb_lock = threading.Lock()
        self._last_frame_time: float | None = None
        self._frames = 0
        self._errors = 0

    @abstractmethod
    def configure(self, config: CameraConfig) -> None: ...

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    def close(self) -> None:
        self.stop()

    def on_frame(self, callback: FrameCallback) -> None:
        with self._cb_lock:
            self._callbacks.append(callback)

    def _emit(self, frame: Frame) -> None:
        self._last_frame_time = time.monotonic()
        self._frames += 1
        with self._cb_lock:
            cbs = list(self._callbacks)
        for cb in cbs:
            try:
                cb(frame)
            except Exception:                    # a consumer bug must not kill the capture thread
                self._errors += 1
                log.exception("frame callback failed")

    def health(self) -> dict[str, Any]:
        age = None if self._last_frame_time is None else round(time.monotonic() - self._last_frame_time, 3)
        return {"name": self.name, "frames": self._frames, "frame_age_s": age, "callback_errors": self._errors}


class Picamera2Camera(Camera):
    """IMX296 (mono, global shutter) via picamera2 on a Raspberry Pi 5.

    System setup (MUST be verified on the real modules - written from the
    Raspberry Pi documentation of the Global Shutter camera, not tested here):

    /boot/firmware/config.txt::

        camera_auto_detect=0
        dtoverlay=imx296,always-on          # camera on CAM1 / the second CSI port
        dtoverlay=imx296,always-on,cam0     # camera on CAM0

    External trigger mode is a module parameter of the imx296 driver, set
    before the camera is opened (as root), e.g. in a systemd ExecStartPre::

        echo 1 > /sys/module/imx296/parameters/trigger_mode

    In trigger mode the sensor waits for the XTR pin; exposure = low-pulse
    width + ~14 us. The official GS camera needs a board modification to bring
    XTR out; third-party modules differ (pin, 1.8/3.3 V level, polarity,
    pull-up) - see hardware/electronics/README.md.

    Without trigger pulses no frames arrive and libcamera may time out after
    a few seconds; the capture thread tolerates that (``timeout`` errors are
    counted, not fatal).
    """

    def __init__(self, name: str, camera_num: int, config: CameraConfig | None = None) -> None:
        super().__init__(name)
        self.camera_num = camera_num
        self._cfg = config or CameraConfig()
        self._cam: Any = None
        self._thread: threading.Thread | None = None
        self._run = False
        self._unpack: Callable[[np.ndarray], np.ndarray] | None = None

    @staticmethod
    def set_trigger_mode(enabled: bool) -> None:
        path = "/sys/module/imx296/parameters/trigger_mode"
        with open(path, "w") as f:                # needs root (or a udev rule)
            f.write("1" if enabled else "0")

    def configure(self, config: CameraConfig) -> None:
        self._cfg = config
        if self._cam is not None:
            self.stop()
            self.start()

    def start(self) -> None:
        from picamera2 import Picamera2           # optional dependency, Pi only

        cfg = self._cfg
        cam = Picamera2(self.camera_num)
        sensor_fmt = None
        for m in cam.sensor_modes:
            if tuple(m["size"]) == tuple(cfg.size):
                sensor_fmt = m["unpacked"] if "unpacked" in m else m["format"]
        raw = {"size": cfg.size}
        if sensor_fmt:
            raw["format"] = str(sensor_fmt)
        controls: dict[str, Any] = {"AeEnable": False, "AwbEnable": False, "AnalogueGain": cfg.analogue_gain,
                                    "ExposureTime": cfg.exposure_us}
        if cfg.crop:
            controls["ScalerCrop"] = cfg.crop
        conf = cam.create_video_configuration(raw=raw, main={"size": (64, 64)}, buffer_count=cfg.buffer_count,
                                              controls=controls, queue=False)
        cam.configure(conf)
        fmt = str(cam.camera_configuration()["raw"]["format"])
        self._unpack = _unpacker(fmt)
        cam.start()
        self._cam = cam
        self._run = True
        self._thread = threading.Thread(target=self._loop, name=f"cam-{self.name}", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        seq = 0
        while self._run:
            try:
                req = self._cam.capture_request(wait=2.0)
            except Exception:                      # timeout while waiting for a trigger
                self._errors += 1
                continue
            if req is None:
                continue
            try:
                arr = req.make_array("raw")
                md = req.get_metadata()
            finally:
                req.release()
            img = self._unpack(arr) if self._unpack else arr
            seq += 1
            self._emit(Frame(self.name, seq, int(md.get("SensorTimestamp", time.monotonic_ns())), img,
                             black_level=float(md.get("SensorBlackLevels", [0])[0]) / 64.0, full_scale=1023,
                             meta={"exposure_us": md.get("ExposureTime"), "gain": md.get("AnalogueGain")}))

    def stop(self) -> None:
        self._run = False
        if self._thread:
            self._thread.join(timeout=3)
            self._thread = None
        if self._cam is not None:
            try:
                self._cam.stop()
                self._cam.close()
            finally:
                self._cam = None

    def health(self) -> dict[str, Any]:
        h = super().health()
        h.update({"backend": "picamera2", "camera_num": self.camera_num, "running": self._cam is not None})
        return h


def _unpacker(fmt: str) -> Callable[[np.ndarray], np.ndarray] | None:
    """Raw buffer -> uint16 (H, W). Handles unpacked 16-bit and MIPI CSI-2 packed 10-bit."""
    if "CSI2P" in fmt and "10" in fmt:
        def unpack(a: np.ndarray) -> np.ndarray:
            a = a.astype(np.uint16)
            w5 = (a.shape[1] // 5) * 5
            b = a[:, :w5].reshape(a.shape[0], -1, 5)
            lo = b[:, :, 4]
            out = np.stack([(b[:, :, k] << 2) | ((lo >> (2 * k)) & 3) for k in range(4)], -1)
            return out.reshape(a.shape[0], -1)
        return unpack

    def view16(a: np.ndarray) -> np.ndarray:
        v = a.view(np.uint16) if a.dtype == np.uint8 else a.astype(np.uint16)
        return v >> 6 if v.max(initial=0) > 1023 else v   # left-aligned 10-bit
    return view16
