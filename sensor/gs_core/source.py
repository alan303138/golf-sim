"""ShotSource implementations for gs_service.

``PipelineShotSource``: the real measurement pipeline with injected cameras,
McuLink and calibration (objects, or ``module:attr`` entry-point strings; this
package never imports gs_mcu).

``SyntheticShotSource``: the same pipeline on rendered frames with an
internal fake MCU - load it as ``gs_core.source:SyntheticShotSource``.

Life cycle (burst / vision policies)::

    disarmed --arm--> WAIT_BALL: MCU in free_run, 1 pulse per frame, ~3 fps;
                      each frame pair is searched for the ball at rest
              ball seen in 2 consecutive pairs (same place, +-2 mm)
           --> READY: MCU configured for a burst (strobe interval from the speed
                      hint), armed; no frames flow
              ShotTrigger + burst frames
           --> PROCESSING (worker thread) --> shot --> WAIT_BALL

free_run policy: frames flow continuously; a quick check per frame pair looks
for ball images away from the rest position and then processes that pair
(+ the next one).
"""
from __future__ import annotations

import asyncio
import importlib
import logging
import os
import threading
import time
from collections import deque
from typing import Any, AsyncIterator

import numpy as np
from gs_common import DeviceState, DeviceStatus, McuLink, Shot, ShotSource, ShotTrigger, StrobeConfig

from .calibration import RigCalibration
from .camera import Camera, Frame
from .capture import CLUB_SPEED_HINT, POLICIES, CapturePolicy, pulse_offsets
from .pipeline import ProcessorConfig, ShotProcessor, ShotResult, detect_rest

log = logging.getLogger(__name__)

PREVIEW = StrobeConfig(mode="free_run", strobe_width_us=10, strobe_interval_us=1000, pulses_per_exposure=1,
                       first_pulse_delay_us=50, exposure_tail_us=50, burst_exposures=1, frame_period_us=330_000)


def load_object(spec: Any) -> Any:
    """'package.module:attr' -> object (called if it is a class/factory); objects pass through."""
    if not isinstance(spec, str):
        return spec
    mod, _, attr = spec.partition(":")
    obj: Any = importlib.import_module(mod)
    for part in attr.split("."):
        obj = getattr(obj, part)
    return obj() if callable(obj) else obj


class PipelineShotSource(ShotSource):
    def __init__(self, cameras: list[Camera] | str | None = None, mcu: McuLink | str | None = None,
                 calibration: RigCalibration | str | None = None, policy: str | CapturePolicy = "burst",
                 processor: ProcessorConfig | None = None, frame_timeout_s: float = 2.0,
                 trigger_camera: Camera | None = None) -> None:
        cameras = cameras if cameras is not None else os.environ.get("GS_CAMERAS", "gs_core.source:pi_cameras")
        mcu = mcu if mcu is not None else os.environ.get("GS_MCU")
        calibration = calibration if calibration is not None else os.environ.get(
            "GS_CALIBRATION", "/var/lib/golfsim/calibration.json")
        self.cameras: list[Camera] = list(load_object(cameras))
        if mcu is None:
            raise ValueError("PipelineShotSource needs an McuLink (object or entry point, or GS_MCU)")
        self.mcu: McuLink = load_object(mcu)
        self.calib_error = ""
        if isinstance(calibration, str):
            try:
                calibration = RigCalibration.load(calibration)
            except Exception as e:                           # reported in health, source stays in ERROR
                self.calib_error = f"{type(e).__name__}: {e}"
                calibration = None
        self.rig: RigCalibration | None = calibration
        self.policy: CapturePolicy = POLICIES[policy]() if isinstance(policy, str) else policy
        self.proc_cfg = processor or ProcessorConfig()
        self.processor = ShotProcessor(self.rig, self.proc_cfg) if self.rig else None
        self.frame_timeout_s = frame_timeout_s
        self._frames: dict[str, deque[Frame]] = {c.name: deque(maxlen=32) for c in self.cameras}
        self._frame_lock = threading.Lock()
        self._frame_event: asyncio.Event | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._triggers: asyncio.Queue[tuple[ShotTrigger, float]] | None = None
        self._shots: asyncio.Queue[Shot] | None = None
        self._task: asyncio.Task | None = None
        self._state = DeviceState.IDLE
        self._armed = False
        self._mode = "idle"
        self._rest: np.ndarray | None = None
        self._rest_info: dict[str, Any] = {}
        self._message = ""
        self._speed_hint: float | None = None
        self._club: str | None = None
        self._strobe: StrobeConfig = PREVIEW
        self._last: dict[str, Any] = {}
        self._counters = {"shots": 0, "failed": 0, "frame_mismatch": 0, "trigger_without_frames": 0}
        self.last_result: ShotResult | None = None
        # vision policy: a separate free-running camera delivering small crops around the ball
        self.trigger_camera = trigger_camera
        self._crop_rest: np.ndarray | None = None

    # ------------------------------------------------------------- ShotSource
    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._frame_event = asyncio.Event()
        self._triggers = asyncio.Queue()
        self._shots = asyncio.Queue()
        for c in self.cameras:
            c.on_frame(self._on_frame)
        self.mcu.on_trigger(self._on_trigger)
        if self.trigger_camera is not None:
            self.trigger_camera.on_frame(self._on_trigger_crop)
            await asyncio.to_thread(self.trigger_camera.start)
        await asyncio.to_thread(self.mcu.open)
        for c in self.cameras:
            await asyncio.to_thread(c.start)
        self._task = asyncio.create_task(self._run(), name="gs-core-pipeline")
        if self.rig is None:
            self._set(DeviceState.ERROR, "未載入校正檔 (no calibration)")

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
            self._task = None
        try:
            await asyncio.to_thread(self.mcu.arm, False)
        except Exception:
            pass
        for c in self.cameras + ([self.trigger_camera] if self.trigger_camera else []):
            await asyncio.to_thread(c.stop)
        await asyncio.to_thread(self.mcu.close)

    async def arm(self) -> None:
        self._armed = True
        if self.rig is not None:
            await self._enter_wait_ball()

    async def disarm(self) -> None:
        self._armed = False
        self._mode = "idle"
        await asyncio.to_thread(self.mcu.arm, False)
        if self.rig is not None:
            self._set(DeviceState.IDLE, "未啟用")

    def status(self) -> DeviceStatus:
        return DeviceStatus(state=self._state, ball_detected=self._rest is not None, message=self._message,
                            health=self.health())

    async def shots(self) -> AsyncIterator[Shot]:
        while self._shots is None:
            await asyncio.sleep(0.05)
        while True:
            yield await self._shots.get()

    def apply_config(self, config: dict[str, Any]) -> None:
        club = config.get("club") or config.get("selected_club")
        if isinstance(club, str) and club.lower() in CLUB_SPEED_HINT:
            if club != self._club:
                self._speed_hint = None
            self._club = club.lower()
        hint = config.get("speed_hint_mps")
        if isinstance(hint, (int, float)) and hint > 0:
            self._speed_hint = float(hint)
        marking = config.get("ball_marking")
        if marking in ("none", "line", "dots", "logo"):
            self.proc_cfg.spin.marked = marking in ("line", "dots")
        pol = config.get("capture_policy")
        if isinstance(pol, str) and pol in POLICIES and pol != self.policy.name:
            self.policy = POLICIES[pol]()

    def fw_version(self) -> str | None:
        try:
            return self.mcu.info().fw_version
        except Exception:
            return None

    def health(self) -> dict[str, Any]:
        h: dict[str, Any] = {}
        for c in self.cameras:
            try:
                h[f"camera_{c.name}"] = c.health()
            except Exception as e:
                h[f"camera_{c.name}"] = {"ok": False, "error": str(e)}
        h["calibration"] = ({"ok": True, **self.rig.summary()} if self.rig else {"ok": False, "error": self.calib_error})
        try:
            h["mcu"] = self.mcu.status()
        except Exception as e:
            h["mcu"] = {"ok": False, "error": str(e)}
        h["pipeline"] = {"policy": self.policy.name, "mode": self._mode, "speed_hint_mps": self.speed_hint(),
                         "strobe_interval_us": self._strobe.strobe_interval_us,
                         "pulses_per_exposure": self._strobe.pulses_per_exposure,
                         "rest_position_mm": None if self._rest is None else (self._rest * 1e3).round(1).tolist(),
                         **self._counters, "last": self._last}
        return h

    # -------------------------------------------------------------- internals
    def speed_hint(self) -> float | None:
        if self._speed_hint:
            return self._speed_hint
        return CLUB_SPEED_HINT.get(self._club or "", None)

    def _set(self, state: DeviceState, msg: str) -> None:
        self._state, self._message = state, msg

    def _on_frame(self, f: Frame) -> None:                      # camera thread
        with self._frame_lock:
            self._frames.setdefault(f.camera, deque(maxlen=32)).append(f)
        if self._loop and self._frame_event:
            self._loop.call_soon_threadsafe(self._frame_event.set)

    def _on_trigger(self, t: ShotTrigger) -> None:             # MCU reader thread
        if self._loop and self._triggers:
            self._loop.call_soon_threadsafe(self._triggers.put_nowait, (t, time.monotonic()))

    def _drop_frames_before(self, t_host: float) -> None:
        """Frames delivered before the trigger report belong to an earlier exposure
        (preview frames still in flight when the MCU was reconfigured)."""
        with self._frame_lock:
            for d in self._frames.values():
                while d and d[0].host_time < t_host:
                    d.popleft()

    def _on_trigger_crop(self, f: Frame) -> None:          # trigger-camera thread (vision policy)
        """Fire the burst from the capture thread itself: every ms counts (see
        capture.vision_trigger_budget)."""
        from .capture import VisionTriggerPolicy
        if not isinstance(self.policy, VisionTriggerPolicy) or self._mode != "ready":
            self._crop_rest = None
            return
        if self._crop_rest is None:
            self._crop_rest = f.image
            return
        if VisionTriggerPolicy.departed(f.image, self._crop_rest):
            self._crop_rest = None
            self.mcu.fire()

    def _clear_frames(self) -> None:
        with self._frame_lock:
            for d in self._frames.values():
                d.clear()

    async def _configure(self, cfg: StrobeConfig, arm: bool) -> None:
        self._strobe = cfg
        await asyncio.to_thread(self.mcu.arm, False)
        await asyncio.to_thread(self.mcu.configure, cfg)
        self._clear_frames()
        if arm:
            await asyncio.to_thread(self.mcu.arm, True)

    async def _enter_wait_ball(self) -> None:
        self._rest = None
        self._mode = "wait_ball"
        self._set(DeviceState.READY, "請放球 (waiting for ball)")
        if self.policy.mcu_mode == "free_run":
            await self._configure(self.policy.strobe_config(self.speed_hint()), True)
        else:
            await self._configure(PREVIEW, True)

    async def _enter_ready(self) -> None:
        self._mode = "ready"
        self._set(DeviceState.READY, "就緒，等待擊球")
        if self.policy.mcu_mode != "free_run":
            await self._configure(self.policy.strobe_config(self.speed_hint()), True)

    def _pop_pairs(self) -> list[dict[str, Frame]]:
        """Frame pairs (one per camera, same exposure) in time order."""
        names = [c.name for c in self.cameras]
        out = []
        with self._frame_lock:
            while all(self._frames[n] for n in names):
                fs = {n: self._frames[n][0] for n in names}
                ts = [f.timestamp_ns for f in fs.values()]
                if max(ts) - min(ts) > 3_000_000:              # not the same trigger: drop the oldest
                    oldest = min(fs, key=lambda n: fs[n].timestamp_ns)
                    self._frames[oldest].popleft()
                    self._counters["frame_mismatch"] += 1
                    continue
                for n in names:
                    self._frames[n].popleft()
                out.append(fs)
        return out

    async def _run(self) -> None:
        assert self._frame_event and self._triggers
        prev_rest: np.ndarray | None = None
        while True:
            if not self._armed or self.rig is None:
                await asyncio.sleep(0.1)
                self._clear_frames()
                continue
            if self._mode == "ready" and self.policy.mcu_mode != "free_run":
                try:
                    trig, t_host = await asyncio.wait_for(self._triggers.get(), timeout=0.25)
                except asyncio.TimeoutError:
                    continue
                self._drop_frames_before(t_host - 0.002)
                await self._handle_trigger(trig)
                prev_rest = None
                continue
            # frame-driven modes (wait_ball preview, free-run capture)
            try:
                await asyncio.wait_for(self._frame_event.wait(), timeout=0.25)
            except asyncio.TimeoutError:
                continue
            self._frame_event.clear()
            for pair in self._pop_pairs():
                if self._mode == "wait_ball":
                    X, info = await asyncio.to_thread(detect_rest, {n: f.image for n, f in pair.items()}, self.rig)
                    if X is not None and prev_rest is not None and np.linalg.norm(X - prev_rest) < 0.002:
                        self._rest, self._rest_info = 0.5 * (X + prev_rest), info
                        await self._enter_ready()
                        break
                    prev_rest = X
                elif self._mode == "ready" and self.policy.mcu_mode == "free_run":
                    if await asyncio.to_thread(self._moving_ball, pair):
                        await self._handle_free_run(pair)
                        prev_rest = None
                        break

    def _moving_ball(self, pair: dict[str, Frame]) -> bool:
        """Cheap test: bright blobs far from the rest position in camera 0 (1/8 scale)."""
        import cv2
        cam = self.rig.cam0
        f = pair[cam.name].image.astype(np.float32)
        small = cv2.resize(f, None, fx=0.125, fy=0.125, interpolation=cv2.INTER_AREA)
        bg = float(np.median(small))
        mask = small > bg + 0.5 * (float(np.percentile(small, 99.9)) - bg)
        if self._rest is not None:
            u, v = cam.project(self._rest) / 8
            r = 0.9 * cam.fx * 0.02134 / np.linalg.norm(self._rest - cam.center) / 8
            yy, xx = np.ogrid[: small.shape[0], : small.shape[1]]
            mask &= (xx - u) ** 2 + (yy - v) ** 2 > (1.6 * r) ** 2
        return int(mask.sum()) > 40

    async def _collect(self, n: int, timeout: float) -> dict[str, list[Frame]]:
        assert self._frame_event
        names = [c.name for c in self.cameras]
        deadline = time.monotonic() + timeout
        while True:
            with self._frame_lock:
                if all(len(self._frames[x]) >= n for x in names):
                    return {x: [self._frames[x].popleft() for _ in range(n)] for x in names}
            left = deadline - time.monotonic()
            if left <= 0:
                with self._frame_lock:
                    return {x: list(self._frames[x]) for x in names}
            self._frame_event.clear()
            try:
                await asyncio.wait_for(self._frame_event.wait(), timeout=left)
            except asyncio.TimeoutError:
                pass

    async def _handle_trigger(self, trig: ShotTrigger) -> None:
        cfg = self._strobe
        n = len(trig.exposure_start_us)
        self._set(DeviceState.PROCESSING, "運算中")
        frames = await self._collect(n, self.frame_timeout_s + n * cfg.frame_period_us * 1e-6)
        got = min(len(v) for v in frames.values())
        if got == 0:
            self._counters["trigger_without_frames"] += 1
            self._last = {"error": "trigger without frames"}
            await self._enter_wait_ball()
            return
        # check that the frame spacing matches the MCU exposure spacing (wrong pairing otherwise)
        for name, fl in frames.items():
            if len(fl) >= 2 and len(trig.exposure_start_us) >= 2:
                d_cam = (fl[1].timestamp_ns - fl[0].timestamp_ns) / 1000
                d_mcu = trig.exposure_start_us[1] - trig.exposure_start_us[0]
                if abs(d_cam - d_mcu) > 2000:
                    self._counters["frame_mismatch"] += 1
        if got < n:
            trig = ShotTrigger(trig.seq, trig.t_us, trig.source, trig.exposure_start_us[:got], trig.pulse_offsets_us)
        await self._process(frames, trig, cfg)

    async def _handle_free_run(self, pair: dict[str, Frame]) -> None:
        cfg = self._strobe
        self._set(DeviceState.PROCESSING, "運算中")
        nxt = await self._collect(1, 2 * cfg.frame_period_us * 1e-6 + 0.5)
        frames = {n: [pair[n]] + nxt.get(n, [])[:1] for n in pair}
        k = min(len(v) for v in frames.values())
        frames = {n: v[:k] for n, v in frames.items()}
        f0 = frames[self.cameras[0].name]
        starts = [int(f.timestamp_ns // 1000) for f in f0]
        trig = ShotTrigger(0, starts[0], "free_run", starts, pulse_offsets(cfg))
        await self._process(frames, trig, cfg)

    async def _process(self, frames: dict[str, list[Frame]], trig: ShotTrigger, cfg: StrobeConfig) -> None:
        assert self.processor and self._shots is not None
        try:
            res = await asyncio.to_thread(self.processor.process, frames, trig, cfg, self._rest)
        except Exception as e:                                  # never let one bad shot kill the source
            log.exception("shot processing failed")
            res = None
            self._last = {"error": f"{type(e).__name__}: {e}"}
        self.last_result = res
        if res is not None:
            self._last = {"timings_s": {k: round(v, 3) for k, v in res.timings.items()}, "reason": res.reason}
        if res is not None and res.shot is not None:
            self._counters["shots"] += 1
            if res.launch is not None:
                self._last["spin_confidence"] = None if res.spin is None else round(res.spin.confidence, 3)
                self._speed_hint = res.launch.speed_mps            # next shot likely similar
            self._on_shot(res)
            self._shots.put_nowait(res.shot)
        else:
            self._counters["failed"] += 1
        if self._armed:
            await self._enter_wait_ball()
        else:
            self._set(DeviceState.IDLE, "未啟用")

    def _on_shot(self, res: ShotResult) -> None:
        """Hook for subclasses."""


def pi_cameras() -> list[Camera]:
    """The two IMX296 cameras on the Pi 5 CSI ports (cam0 = CSI0 / up-range side)."""
    from .camera import Picamera2Camera
    return [Picamera2Camera("cam0", 0), Picamera2Camera("cam1", 1)]


# ------------------------------------------------------------------ synthetic
class SyntheticShotSource(PipelineShotSource):
    """Full pipeline on rendered frames. No-argument construction works.

    ``inject(truth)`` queues a ground-truth shot; it is struck as soon as the
    source is READY (ball detected, burst armed). With ``auto_interval_s`` > 0
    (or env GS_SYNTH_SHOT_INTERVAL_S) random realistic shots are struck on a
    timer while armed. A new ball is placed ``reload_s`` after each shot.
    The pipeline sees a calibration with realistic errors, not the true rig.
    """

    def __init__(self, auto_interval_s: float | None = None, seed: int = 0, ball_marking: str = "logo",
                 calibration_error: float = 1.0, policy: str = "burst", reload_s: float = 0.5,
                 trigger_delay_s: float = 0.0006) -> None:
        from .synthetic.ball import BallSpec
        from .synthetic.rig import FakeMcuLink, SyntheticCamera, SyntheticScene
        from .synthetic.scene import perturbed_rig

        if auto_interval_s is None:
            auto_interval_s = float(os.environ.get("GS_SYNTH_SHOT_INTERVAL_S", "0") or 0)
        self.auto_interval_s = auto_interval_s
        self.rng = np.random.default_rng(seed)
        self.scene = SyntheticScene(ball=BallSpec(marking=ball_marking), seed=seed)
        cams = [SyntheticCamera(c.name, self.scene) for c in self.scene.true_rig.pair]
        self.fake_mcu = FakeMcuLink(cams, self.scene)
        est = (perturbed_rig(self.scene.true_rig, np.random.default_rng(seed + 1000), calibration_error)
               if calibration_error > 0 else self.scene.true_rig)
        super().__init__(cams, self.fake_mcu, est, policy, frame_timeout_s=60.0)
        if ball_marking in ("line", "dots"):
            self.proc_cfg.spin.marked = True
        self.reload_s = reload_s
        self.trigger_delay_s = trigger_delay_s
        self._pending: deque = deque()
        self.results: list[tuple[Any, ShotResult]] = []     # (truth, result) for every processed shot
        self._current_truth = None
        self._in_flight = False
        self._aux: asyncio.Task | None = None

    def inject(self, truth=None, club: str | None = None) -> None:
        """Queue a ground-truth shot (``synthetic.scene.ShotTruth``; None = random).

        ``club`` simulates the golfer selecting a club in the app before the
        shot (it sets the strobe speed hint, exactly like ``apply_config``).
        Random shots select the club matching their type, as a user would."""
        from .synthetic.scene import random_shot
        if truth is None:
            truth = random_shot(self.rng)
            club = club or {"driver": "driver", "wood": "wood", "mid_iron": "iron", "short_iron": "iron",
                            "wedge": "wedge", "chip": "chip", "putt": "putter"}.get(truth.label)
        self._pending.append((truth, club))

    emit = inject

    async def start(self) -> None:
        await super().start()
        self._aux = asyncio.create_task(self._driver(), name="gs-synthetic-driver")

    async def stop(self) -> None:
        if self._aux:
            self._aux.cancel()
            self._aux = None
        await super().stop()

    async def _driver(self) -> None:
        last_auto = time.monotonic()
        while True:
            await asyncio.sleep(0.05)
            if self.scene.state.rest is None or self.scene.state.flight is not None:
                if self._mode in ("wait_ball", "idle") and self._state != DeviceState.PROCESSING:
                    await asyncio.sleep(self.reload_s)
                    off = self.rng.normal(0, 0.003, 3) * np.array([1, 1, 0])
                    self.scene.place_ball(off)
            if self._mode != "ready" or not self._armed or self._in_flight:
                continue
            if not self._pending and self.auto_interval_s > 0 and time.monotonic() - last_auto > self.auto_interval_s:
                self.inject()
            if self._pending:
                truth, club = self._pending[0]
                if club and club != self._club:
                    self.apply_config({"club": club})
                    await self._enter_ready()              # re-plan the strobe for the new hint
                self._pending.popleft()
                last_auto = time.monotonic()
                self._current_truth = truth
                self._in_flight = True
                self.fake_mcu.hardware_trigger(truth, self.trigger_delay_s)

    def _on_shot(self, res: ShotResult) -> None:
        if res.shot is not None:
            res.shot.quality.sources = ["stereo", "synthetic"]
        self.results.append((self._current_truth, res))

    async def _process(self, frames, trig, cfg) -> None:
        try:
            await super()._process(frames, trig, cfg)
            if self.last_result is None or self.last_result.shot is None:
                self.results.append((self._current_truth, self.last_result))
        finally:
            self._in_flight = False

    async def _handle_trigger(self, trig) -> None:
        try:
            await super()._handle_trigger(trig)
        finally:
            self._in_flight = False
