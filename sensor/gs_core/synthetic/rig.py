"""Synthetic stereo rig: two SyntheticCameras and a fake Pico McuLink that
together behave like the real hardware (frames on trigger, ShotTrigger
reports with exact pulse timing), driven by an injected ground-truth shot."""
from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np
from gs_common import McuInfo, McuLink, ShotTrigger, StrobeConfig

from ..calibration import RigCalibration
from ..camera import Camera, CameraConfig, Frame
from ..capture import pulse_offsets
from .ball import BallModel, BallSpec
from .render import Renderer, SensorSpec
from .scene import ClubMotion, Flight, RigSpec, ShotTruth, make_rig, random_orientation


@dataclass
class SceneState:
    flight: Flight | None = None          # None: ball at rest at ``rest`` (or absent)
    club: ClubMotion | None = None
    rest: np.ndarray | None = None        # None: no ball on the mat
    orientation: np.ndarray | None = None


class SyntheticScene:
    """Shared world: the true rig, the ball model and what is happening (thread-safe)."""

    def __init__(self, rig: RigCalibration | None = None, ball: BallSpec | None = None,
                 sensor: SensorSpec | None = None, seed: int = 0) -> None:
        self.true_rig = rig or make_rig(RigSpec())
        self.ball = BallModel(ball or BallSpec())
        self.rng = np.random.default_rng(seed)
        led_axis = np.asarray(self.true_rig.meta.get("led_axis", [0, 1, 0]), float)
        self.renderers = {c.name: Renderer(c, self.true_rig.lights, led_axis, self.ball, sensor, seed=seed + i)
                          for i, c in enumerate(self.true_rig.pair)}
        self._lock = threading.Lock()
        self.state = SceneState(rest=np.zeros(3), orientation=random_orientation(self.rng))
        self.sensor = next(iter(self.renderers.values())).s

    def place_ball(self, rest: np.ndarray | None = None) -> None:
        with self._lock:
            self.state = SceneState(rest=np.zeros(3) if rest is None else np.asarray(rest, float),
                                    orientation=random_orientation(self.rng))

    def remove_ball(self) -> None:
        with self._lock:
            self.state = SceneState(rest=None)

    def launch(self, truth: ShotTruth, t_impact: float) -> None:
        with self._lock:
            st = self.state
            rest = np.zeros(3) if st.rest is None else st.rest
            fl = Flight(truth, t_impact, rest=rest, orientation0=st.orientation)
            club = ClubMotion(truth.club_speed, t_impact, rest=rest) if truth.club and not truth.rolling else None
            self.state = SceneState(flight=fl, club=club, rest=rest, orientation=st.orientation)

    def ball_state(self, t: float):
        st = self.state
        if st.flight is not None:
            return st.flight.position(t), st.flight.orientation(t)
        if st.rest is None:
            return None
        return st.rest, st.orientation

    def render(self, cam: str, pulse_times: list[float], width_s: float) -> np.ndarray:
        r = self.renderers[cam]
        st = self.state
        e = r.expose_electrons(pulse_times, width_s, self.ball_state, st.club)
        return r.to_raw(e)


class SyntheticCamera(Camera):
    """Renders a frame whenever the fake MCU triggers it (external-trigger behaviour)."""

    def __init__(self, name: str, scene: SyntheticScene) -> None:
        super().__init__(name)
        self.scene = scene
        self._cfg = CameraConfig()
        self._running = False
        self._seq = 0

    def configure(self, config: CameraConfig) -> None:
        self._cfg = config

    def start(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False

    def expose(self, start_s: float, pulse_times: list[float], width_s: float) -> Frame | None:
        """Called by the fake MCU for each trigger pulse (times in seconds, MCU clock)."""
        if not self._running:
            return None
        img = self.scene.render(self.name, pulse_times, width_s)
        self._seq += 1
        f = Frame(self.name, self._seq, int(start_s * 1e9), img, black_level=self.scene.sensor.black_level,
                  full_scale=self.scene.sensor.full_scale)
        self._emit(f)
        return f

    def health(self) -> dict[str, Any]:
        h = super().health()
        h.update({"backend": "synthetic", "running": self._running})
        return h


class FakeMcuLink(McuLink):
    """In-process stand-in for the Pico timing MCU.

    burst:    ``fire()`` (or ``hardware_trigger()``, i.e. TRIG_IN) runs
              ``burst_exposures`` exposures; a ShotTrigger is reported first,
              then the cameras deliver their frames.
    free_run: while armed, one exposure every ``frame_period_us`` (no
              ShotTrigger; frames only), rendered in real time as fast as
              the renderer allows.
    Time base: seconds since construction, reported in microseconds.
    """

    def __init__(self, cameras: list[SyntheticCamera], scene: SyntheticScene) -> None:
        self.cameras = cameras
        self.scene = scene
        self.cfg = StrobeConfig()
        self.armed = False
        self._cbs: list[Callable[[ShotTrigger], None]] = []
        self._seq = 0
        self._t0 = time.monotonic()
        self._q: queue.Queue = queue.Queue()
        self._thread: threading.Thread | None = None
        self._open = False
        self.counters = {"triggers": 0, "exposures": 0}
        self.pending_shot: tuple[ShotTruth, float] | None = None   # (truth, impact->trigger delay s)

    def now_us(self) -> int:
        return int((time.monotonic() - self._t0) * 1e6)

    def open(self) -> None:
        self._open = True
        self._thread = threading.Thread(target=self._loop, name="fake-mcu", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._open = False
        self._q.put(None)
        if self._thread:
            self._thread.join(timeout=10)

    def info(self) -> McuInfo:
        return McuInfo("synthetic-1.0.0", 0, "SYNTH0001", int((time.monotonic() - self._t0) * 1000))

    def configure(self, config: StrobeConfig) -> None:
        self.cfg = config

    def arm(self, armed: bool) -> None:
        self.armed = armed
        if armed and self.cfg.mode == "free_run":
            self._q.put(("free_run",))

    def fire(self) -> None:
        self._q.put(("burst", "software"))

    def hardware_trigger(self, truth: ShotTruth | None = None, trigger_delay_s: float = 0.0006) -> None:
        """Simulate a shot: the ball is struck ``trigger_delay_s`` before TRIG_IN fires
        (acoustic trigger ~0.6 ms). Ignored unless armed in burst mode (like the MCU)."""
        self._q.put(("shot", truth, trigger_delay_s))

    def on_trigger(self, callback: Callable[[ShotTrigger], None]) -> None:
        self._cbs.append(callback)

    def status(self) -> dict[str, Any]:
        return {"ok": self._open, "armed": self.armed, "mode": self.cfg.mode, "faults": [], **self.counters}

    def reboot_to_bootloader(self) -> None:
        raise NotImplementedError("synthetic MCU has no bootloader")

    # ------------------------------------------------------------------
    def _loop(self) -> None:
        while self._open:
            try:
                item = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                return
            if item[0] == "free_run":
                while self._open and self.armed and self.cfg.mode == "free_run":
                    t = self.now_us()
                    self._expose_all(t)
                    time.sleep(max(0.0, self.cfg.frame_period_us * 1e-6 - (self.now_us() - t) * 1e-6))
                continue
            if item[0] == "shot":
                _, truth, delay = item
                if not (self.armed and self.cfg.mode == "burst"):
                    continue
                t_trig = self.now_us()
                if truth is not None:
                    self.scene.launch(truth, t_trig * 1e-6 - delay)
                self._burst(t_trig, "hardware")
            elif item[0] == "burst":
                self._burst(self.now_us(), item[1])

    def _expose_all(self, start_us: int) -> None:
        c = self.cfg
        times = [(start_us + o + 0.5 * c.strobe_width_us) * 1e-6 for o in pulse_offsets(c)]
        for cam in self.cameras:
            cam.expose(start_us * 1e-6, times, c.strobe_width_us * 1e-6)
        self.counters["exposures"] += 1

    def _burst(self, t_trig: int, source: str) -> None:
        c = self.cfg
        self._seq += 1
        self.counters["triggers"] += 1
        starts = [t_trig + c.trigger_delay_us + k * c.frame_period_us for k in range(c.burst_exposures)]
        trig = ShotTrigger(self._seq, t_trig, source, starts, pulse_offsets(c))
        for cb in list(self._cbs):
            cb(trig)
        for s in starts:
            self._expose_all(s)
