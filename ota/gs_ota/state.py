"""Persistent agent state (JSON, written atomically with fsync).

Holds installed versions, anti-rollback watermarks and the in-flight
transaction, so that an update interrupted by a crash or power loss can be
resumed or rolled back on the next start.
"""
from __future__ import annotations

import json
import logging
import threading
from pathlib import Path
from typing import Any

from .fsutil import atomic_write_json

log = logging.getLogger(__name__)

STATE_VERSION = 1


def _default() -> dict[str, Any]:
    return {
        "state_version": STATE_VERSION,
        "installed": {},          # artifact -> version
        "highest_released": {},   # channel -> highest verified manifest 'released'
        "highest_seen": {},       # artifact -> highest version seen in a verified manifest
        "failed_versions": {},    # artifact -> [versions that were rolled back]
        "txn": None,              # in-flight transaction, see engine.py
        "last_result": None,      # {"ok": bool, "message": str, "at": iso}
    }


class StateStore:
    def __init__(self, path: Path):
        self.path = Path(path)
        self._lock = threading.RLock()
        self.data = self._load()

    def _load(self) -> dict[str, Any]:
        if not self.path.exists():
            return _default()
        try:
            data = json.loads(self.path.read_text())
            if not isinstance(data, dict):
                raise ValueError("not an object")
        except (ValueError, OSError) as e:
            # Atomic writes make this unlikely; keep a copy for diagnosis.
            log.error("state file %s unreadable (%s); starting fresh", self.path, e)
            try:
                self.path.rename(self.path.with_suffix(".corrupt"))
            except OSError:
                pass
            return _default()
        base = _default()
        base.update(data)
        return base

    def save(self) -> None:
        with self._lock:
            atomic_write_json(self.path, self.data)

    def update(self, **kw: Any) -> None:
        with self._lock:
            self.data.update(kw)
            self.save()

    @property
    def txn(self) -> dict[str, Any] | None:
        return self.data.get("txn")

    def set_txn(self, txn: dict[str, Any] | None) -> None:
        self.update(txn=txn)

    def txn_update(self, **kw: Any) -> None:
        with self._lock:
            if self.data.get("txn") is None:
                raise RuntimeError("no transaction")
            self.data["txn"].update(kw)
            self.save()
