"""Core orchestration: shot pipeline, status polling, WebSocket fan-out, GSPro, mDNS."""
from __future__ import annotations

import asyncio
import json
import logging
import time
import uuid
from typing import Any

from fastapi import WebSocket

from gs_common import DeviceState, Shot, ShotSource, UpdateState, Updater

from . import __version__
from .config import ConfigStore
from .gspro import GSProAdapter
from .mdns import MdnsAdvertiser
from .settings import Settings
from .store import ShotStore

log = logging.getLogger("gs_service")

_BUSY_PHASES = {"downloading", "verifying", "installing", "rebooting"}


def _load_device_id(settings: Settings) -> str:
    if settings.device_id:
        return settings.device_id
    p = settings.data_dir / "device_id"
    try:
        return p.read_text().strip()
    except OSError:
        did = uuid.uuid4().hex[:8]
        settings.data_dir.mkdir(parents=True, exist_ok=True)
        p.write_text(did)
        return did


class Service:
    def __init__(self, settings: Settings, source: ShotSource, updater: Updater):
        self.settings = settings
        self.source = source
        self.updater = updater
        settings.data_dir.mkdir(parents=True, exist_ok=True)
        self.device_id = _load_device_id(settings)
        self.store = ShotStore(settings.data_dir / "shots.db")
        self.config = ConfigStore(settings.data_dir / "config.json", defaults={
            "gspro_enabled": settings.gspro_enabled,
            "gspro_host": settings.gspro_host,
            "gspro_port": settings.gspro_port,
        })
        self.clients: set[WebSocket] = set()
        self.started_at = time.time()
        self.gspro: GSProAdapter | None = None
        self.mdns: MdnsAdvertiser | None = None
        self._tasks: list[asyncio.Task] = []
        self._loop: asyncio.AbstractEventLoop | None = None
        self._last_status: dict[str, Any] | None = None
        self._last_status_t = 0.0
        self.updater.on_change(self._on_update_change)

    # ------------------------------------------------------------ lifecycle
    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        self.source.apply_config(self.config.get())
        await self.source.start()
        self._tasks = [asyncio.create_task(self._shot_loop()), asyncio.create_task(self._status_loop())]
        await self._reconfigure_gspro()
        if self.settings.mdns:
            self.mdns = MdnsAdvertiser(self.device_id, __version__, self.settings.port)
            await self.mdns.start()

    async def stop(self) -> None:
        for t in self._tasks:
            t.cancel()
        for t in self._tasks:
            try:
                await t
            except (asyncio.CancelledError, Exception):
                pass
        self._tasks = []
        if self.gspro:
            await self.gspro.stop()
        if self.mdns:
            await self.mdns.stop()
        await self.source.stop()
        self.store.close()

    # ------------------------------------------------------------ shots
    async def _shot_loop(self) -> None:
        while True:
            try:
                async for shot in self.source.shots():
                    await self.ingest(shot)
                return
            except asyncio.CancelledError:
                raise
            except Exception:
                log.exception("shot source failed; retrying in 2 s")
                await asyncio.sleep(2)

    async def ingest(self, shot: Shot) -> dict[str, Any]:
        d = shot.to_dict()
        self.store.add(d)
        await self.broadcast(d)
        if self.gspro:
            asyncio.create_task(self.gspro.send_shot(d))
        return d

    # ------------------------------------------------------------ status
    def status_message(self) -> dict[str, Any]:
        st = self.source.status()
        state = st.state.value
        if self.updater.state().phase in _BUSY_PHASES:
            state = DeviceState.UPDATING.value
        return {"type": "status", "state": state, "ball_detected": bool(st.ball_detected),
                "message": st.message or ""}

    def hello_message(self) -> dict[str, Any]:
        return {"type": "hello", "device_id": self.device_id, "sw_version": __version__,
                "fw_version": self.source.fw_version(), "api": 1}

    def status_detail(self) -> dict[str, Any]:
        st = self.source.status()
        msg = self.status_message()
        return {
            "device_id": self.device_id,
            "sw_version": __version__,
            "fw_version": self.source.fw_version(),
            "api": 1,
            "state": msg["state"],
            "ball_detected": msg["ball_detected"],
            "message": msg["message"],
            "health": st.health,
            "uptime_s": round(time.time() - self.started_at, 1),
            "clients": len(self.clients),
            "shots_stored": self.store.count(),
            "source": type(self.source).__name__,
            "updater": type(self.updater).__name__,
            "gspro": self.gspro.info() if self.gspro else {"enabled": False},
        }

    async def _status_loop(self) -> None:
        while True:
            msg = self.status_message()
            now = time.monotonic()
            if msg != self._last_status or now - self._last_status_t >= 2.0:
                self._last_status, self._last_status_t = msg, now
                await self.broadcast(msg)
            await asyncio.sleep(0.2)

    # ------------------------------------------------------------ OTA
    def update_dict(self) -> dict[str, Any]:
        s: UpdateState = self.updater.state()
        return {"sw_version": __version__, "state": s.phase, "phase": s.phase, "progress": s.progress,
                "message": s.message, "current": s.current, "available": s.available, "notes": s.notes,
                "update_available": bool(s.available)}

    def _on_update_change(self, s: UpdateState) -> None:
        msg = {"type": "update", "phase": s.phase, "progress": s.progress, "message": s.message}
        loop = self._loop
        if loop and loop.is_running():
            loop.call_soon_threadsafe(lambda: asyncio.ensure_future(self.broadcast(msg)))

    # ------------------------------------------------------------ config
    def update_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        cfg = self.config.update(patch)
        self.source.apply_config(cfg)
        if self._loop:
            asyncio.ensure_future(self._reconfigure_gspro())
        return cfg

    async def _reconfigure_gspro(self) -> None:
        cfg = self.config.get()
        want = cfg["gspro_enabled"]
        if self.gspro and (not want or (self.gspro.host, self.gspro.port) != (cfg["gspro_host"], cfg["gspro_port"])):
            await self.gspro.stop()
            self.gspro = None
        if want and not self.gspro:
            def ready() -> tuple[bool, bool]:
                st = self.source.status()
                return st.state == DeviceState.READY, st.ball_detected
            self.gspro = GSProAdapter(cfg["gspro_host"], cfg["gspro_port"],
                                      device_id=f"GolfSim-{self.device_id}", status_cb=ready,
                                      shot_number=self.store.count())
            await self.gspro.start()

    # ------------------------------------------------------------ websocket
    async def broadcast(self, msg: dict[str, Any]) -> None:
        if not self.clients:
            return
        text = json.dumps(msg)

        async def send(ws: WebSocket) -> None:
            try:
                await asyncio.wait_for(ws.send_text(text), 3)
            except Exception:
                self.clients.discard(ws)

        await asyncio.gather(*(send(ws) for ws in list(self.clients)))

    async def handle_client_message(self, ws: WebSocket, msg: dict[str, Any]) -> None:
        t = msg.get("type")
        if t == "ping":
            await ws.send_json({"type": "pong"})
        elif t == "arm":
            await self.source.arm()
            await self.broadcast(self.status_message())
        elif t == "disarm":
            await self.source.disarm()
            await self.broadcast(self.status_message())
