"""Built-in mock ShotSource and Updater so the service runs standalone."""
from __future__ import annotations

import asyncio
import random
from typing import Any, AsyncIterator, Callable

from gs_common import (BallData, ClubData, DeviceState, DeviceStatus, Quality, Shot,
                       ShotSource, UpdateState, Updater)

# (name, ball speed m/s, vla, total spin rpm, club speed m/s)
_CLUBS = [
    ("driver", 68.0, 11.5, 2600.0, 45.5),
    ("iron7", 52.0, 17.0, 6500.0, 36.5),
    ("wedge", 38.0, 27.0, 8800.0, 29.0),
]


def random_shot(rng: random.Random | None = None) -> Shot:
    """Plausible random shot (driver / 7-iron / wedge profiles with scatter)."""
    r = rng or random
    _, v, vla, spin, club = r.choice(_CLUBS)
    axis = r.gauss(0, 7)
    return Shot(
        ball=BallData(
            speed_mps=round(max(5.0, r.gauss(v, v * 0.04)), 2),
            vla_deg=round(r.gauss(vla, 1.5), 2),
            hla_deg=round(r.gauss(0, 2.0), 2),
            total_spin_rpm=round(max(300.0, r.gauss(spin, spin * 0.08)), 1),
            spin_axis_deg=round(axis, 2),
        ),
        club=ClubData(speed_mps=round(max(5.0, r.gauss(club, 1.0)), 2)),
        quality=Quality(confidence=round(r.uniform(0.85, 0.99), 2),
                        sources=["synthetic"], spin_measured=False),
    )


class MockShotSource(ShotSource):
    """Emits shots on demand (``emit``) and optionally periodically while armed."""

    def __init__(self, auto_interval: float = 0.0):
        self.auto_interval = auto_interval
        self._q: asyncio.Queue[Shot] = asyncio.Queue()
        self._state = DeviceState.IDLE
        self._armed = False
        self._task: asyncio.Task | None = None
        self._config: dict[str, Any] = {}
        self._processing_until = 0.0

    async def start(self) -> None:
        self._state = DeviceState.IDLE
        if self.auto_interval > 0:
            self._task = asyncio.create_task(self._auto())

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            self._task = None

    async def arm(self) -> None:
        self._armed = True
        self._state = DeviceState.READY

    async def disarm(self) -> None:
        self._armed = False
        self._state = DeviceState.IDLE

    def status(self) -> DeviceStatus:
        state = self._state
        msg = {"idle": "未就緒（已停用）", "ready": "就緒，等待擊球", "processing": "運算中"}.get(state.value, "")
        return DeviceStatus(
            state=state,
            ball_detected=self._armed,
            message=msg,
            health={
                "camera_left": {"ok": True, "fps": 500},
                "camera_right": {"ok": True, "fps": 500},
                "radar": {"ok": True, "noise_floor": 0.012},
                "mcu": {"ok": True, "fw": "mock-1.0.0"},
            },
        )

    def emit(self, shot: Shot | None = None) -> Shot:
        shot = shot or random_shot()
        self._q.put_nowait(shot)
        return shot

    async def shots(self) -> AsyncIterator[Shot]:
        while True:
            shot = await self._q.get()
            prev = self._state
            self._state = DeviceState.PROCESSING
            await asyncio.sleep(0.15)
            self._state = DeviceState.READY if self._armed else prev if prev != DeviceState.PROCESSING else DeviceState.IDLE
            yield shot

    async def _auto(self) -> None:
        while True:
            await asyncio.sleep(self.auto_interval)
            if self._armed:
                self.emit()

    def apply_config(self, config: dict[str, Any]) -> None:
        self._config = dict(config)

    def fw_version(self) -> str | None:
        return "mock-1.0.0"


class MockUpdater(Updater):
    """Pretends a newer sensor build exists; ``apply`` simulates the install phases."""

    def __init__(self, current: str = "0.1.0", available: str = "0.2.0", step_s: float = 0.4):
        self._state = UpdateState(phase="idle", current={"sensor": current, "firmware": "1.0.0"})
        self._available = available
        self._step = step_s
        self._cbs: list[Callable[[UpdateState], None]] = []
        self._task: asyncio.Task | None = None

    def state(self) -> UpdateState:
        return self._state

    def _set(self, **kw: Any) -> None:
        for k, v in kw.items():
            setattr(self._state, k, v)
        for cb in list(self._cbs):
            try:
                cb(self._state)
            except Exception:
                pass

    async def check(self) -> UpdateState:
        self._set(phase="checking", message="檢查更新中")
        await asyncio.sleep(0.1)
        cur = self._state.current.get("sensor", "0")
        if cur != self._available:
            self._set(phase="available", message="有新版本", available={"sensor": self._available},
                      notes="Mock release notes", progress=0.0)
        else:
            self._set(phase="up_to_date", message="已是最新版本", available={}, notes="")
        return self._state

    async def apply(self) -> None:
        if self._task and not self._task.done():
            return
        self._task = asyncio.create_task(self._run())

    async def _run(self) -> None:
        phases = ["downloading", "verifying", "installing", "rebooting"]
        for i, ph in enumerate(phases):
            n = 5 if ph == "downloading" else 1
            for j in range(n):
                self._set(phase=ph, progress=(i + (j + 1) / n) / len(phases), message=ph)
                await asyncio.sleep(self._step)
        cur = dict(self._state.current)
        cur["sensor"] = self._available
        self._set(phase="up_to_date", progress=1.0, message="更新完成", current=cur, available={})

    def on_change(self, callback: Callable[[UpdateState], None]) -> None:
        self._cbs.append(callback)
