"""``AgentUpdater``: gs_common ``Updater`` for gs_service, backed by the OTA
agent's Unix control socket (the agent owns the update; gs_service stays
unprivileged and may be restarted by the update without losing it).

Entry point string for gs_service: ``"gs_ota.client:AgentUpdater"``
(constructor takes an optional socket path; default from ``GS_OTA_SOCKET``
or ``/run/gs-ota/agent.sock``).

on_change callbacks are invoked from a background reader thread; asyncio code
should use ``loop.call_soon_threadsafe`` inside the callback.
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import socket
import threading
import time
from typing import Callable

from gs_common.interfaces import UpdateState, Updater

log = logging.getLogger(__name__)

DEFAULT_SOCKET = "/run/gs-ota/agent.sock"


def _to_state(d: dict) -> UpdateState:
    return UpdateState(phase=str(d.get("phase", "idle")), progress=float(d.get("progress", 0.0)),
                       message=str(d.get("message", "")), current=dict(d.get("current") or {}),
                       available=dict(d.get("available") or {}), notes=str(d.get("notes", "")))


class AgentUnavailable(Exception):
    pass


class AgentUpdater(Updater):
    def __init__(self, socket_path: str | None = None, timeout: float = 120.0):
        self.path = socket_path or os.environ.get("GS_OTA_SOCKET", DEFAULT_SOCKET)
        self.timeout = timeout
        self._state = UpdateState(phase="idle", message="OTA agent not contacted yet")
        self._callbacks: list[Callable[[UpdateState], None]] = []
        self._sub: threading.Thread | None = None
        self._closed = threading.Event()

    # ------------------------------------------------------------------ io
    def _request(self, op: str) -> UpdateState:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        try:
            s.connect(self.path)
            s.sendall((json.dumps({"op": op}) + "\n").encode())
            line = s.makefile("rb").readline()
        except OSError as e:
            raise AgentUnavailable(f"OTA agent unavailable at {self.path}: {e}") from e
        finally:
            s.close()
        resp = json.loads(line) if line else {"ok": False, "error": "no response"}
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error", "agent error"))
        st = _to_state(resp["state"])
        self._update(st)
        return st

    def _update(self, st: UpdateState) -> None:
        self._state = st
        for cb in list(self._callbacks):
            try:
                cb(st)
            except Exception:  # noqa: BLE001
                log.exception("update listener failed")

    def _offline(self, e: Exception) -> UpdateState:
        st = UpdateState(phase="failed", message=str(e), current=dict(self._state.current))
        self._update(st)
        return st

    # ------------------------------------------------------------- Updater
    def state(self) -> UpdateState:
        return self._state

    async def check(self) -> UpdateState:
        try:
            return await asyncio.to_thread(self._request, "check")
        except (AgentUnavailable, RuntimeError) as e:
            return self._offline(e)

    async def apply(self) -> None:
        try:
            await asyncio.to_thread(self._request, "apply")
        except (AgentUnavailable, RuntimeError) as e:
            self._offline(e)

    def on_change(self, callback: Callable[[UpdateState], None]) -> None:
        self._callbacks.append(callback)
        if self._sub is None:
            self._sub = threading.Thread(target=self._subscribe_loop, name="gs-ota-sub",
                                         daemon=True)
            self._sub.start()

    def refresh(self) -> UpdateState:
        try:
            return self._request("state")
        except (AgentUnavailable, RuntimeError) as e:
            return self._offline(e)

    def close(self) -> None:
        self._closed.set()

    def _subscribe_loop(self) -> None:
        backoff = 1.0
        while not self._closed.is_set():
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                s.connect(self.path)
                s.settimeout(60)
                s.sendall(b'{"op": "subscribe"}\n')
                backoff = 1.0
                for line in s.makefile("rb"):
                    if self._closed.is_set():
                        break
                    msg = json.loads(line)
                    if "state" in msg:
                        self._update(_to_state(msg["state"]))
            except (OSError, ValueError) as e:
                log.debug("OTA subscription dropped: %s", e)
            finally:
                s.close()
            self._closed.wait(backoff)
            backoff = min(backoff * 2, 30.0)
            time.sleep(0)
