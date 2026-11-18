"""``OtaUpdater``: the gs_common ``Updater`` interface over an UpdateEngine.

Runs in the OTA agent process (it owns the transaction and survives the
service restart it triggers). gs_service talks to it through
``gs_ota.client.AgentUpdater``, which implements the same interface over the
agent's Unix socket. In tests and single-process setups OtaUpdater can be
used directly.

Callbacks registered with ``on_change`` are invoked from worker threads;
asyncio users should hop to their loop with ``loop.call_soon_threadsafe``.
"""
from __future__ import annotations

import asyncio
import logging
import threading
from typing import Callable

from gs_common.interfaces import UpdateState, Updater

from .engine import UpdateEngine, UpdateError

log = logging.getLogger(__name__)


class OtaUpdater(Updater):
    def __init__(self, engine: UpdateEngine):
        self.engine = engine
        self._apply_thread: threading.Thread | None = None
        self._periodic: threading.Thread | None = None
        self._stop = threading.Event()

    # Updater interface ----------------------------------------------------
    def state(self) -> UpdateState:
        return self.engine.state()

    async def check(self) -> UpdateState:
        if self.busy:
            return self.engine.state()
        return await asyncio.to_thread(self.engine.check)

    async def apply(self) -> None:
        self.start_apply()

    def on_change(self, callback: Callable[[UpdateState], None]) -> None:
        self.engine.on_change(callback)

    # extras ------------------------------------------------------------------
    @property
    def busy(self) -> bool:
        return self._apply_thread is not None and self._apply_thread.is_alive()

    def start_apply(self, auto: bool = False) -> threading.Thread:
        if self.busy:
            raise UpdateError("an update is already running")
        t = threading.Thread(target=self._apply_worker, args=(auto,), name="gs-ota-apply",
                             daemon=True)
        self._apply_thread = t
        t.start()
        return t

    def _apply_worker(self, auto: bool) -> None:
        try:
            self.engine.apply(auto=auto)
        except UpdateError as e:
            log.warning("apply refused: %s", e)

    def wait(self, timeout: float | None = None) -> None:
        if self._apply_thread:
            self._apply_thread.join(timeout)

    def start_periodic(self, interval_s: float, auto_apply: bool = False,
                       initial_delay_s: float = 60.0) -> None:
        """Background check every ``interval_s`` seconds (optionally applying)."""
        if interval_s <= 0 or self._periodic:
            return

        def loop() -> None:
            delay = initial_delay_s
            while not self._stop.wait(delay):
                delay = interval_s
                if self.busy:
                    continue
                try:
                    st = self.engine.check()
                    if auto_apply and st.phase == "available":
                        self.start_apply(auto=True)
                except Exception:  # noqa: BLE001
                    log.exception("periodic update check failed")
        self._periodic = threading.Thread(target=loop, name="gs-ota-periodic", daemon=True)
        self._periodic.start()

    def close(self) -> None:
        self._stop.set()
