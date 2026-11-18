"""A/B slot layout for the sensor software.

    <root>/slots/a/          unpacked release (bin/, lib/, share/, release.json)
    <root>/slots/b/
    <root>/current   -> slots/a   what gs-service.service runs
    <root>/committed -> slots/a   last release that passed the health check;
                                  the OTA agent itself runs from here, so it
                                  never executes unconfirmed code.

Both symlinks are relative and are switched with rename(2) (atomic).
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

from .fsutil import atomic_symlink, rmtree

SLOTS = ("a", "b")


class SlotError(Exception):
    pass


class SlotManager:
    def __init__(self, root: str | Path):
        self.root = Path(root)
        self.slots_dir = self.root / "slots"
        self.current_link = self.root / "current"
        self.committed_link = self.root / "committed"

    def ensure_layout(self) -> None:
        for s in SLOTS:
            (self.slots_dir / s).mkdir(parents=True, exist_ok=True)

    def slot_path(self, slot: str) -> Path:
        if slot not in SLOTS:
            raise SlotError(f"bad slot {slot!r}")
        return self.slots_dir / slot

    def _read_link(self, link: Path) -> str | None:
        try:
            target = os.readlink(link)
        except OSError:
            return None
        name = Path(target).name
        return name if name in SLOTS else None

    def current(self) -> str | None:
        return self._read_link(self.current_link)

    def committed(self) -> str | None:
        return self._read_link(self.committed_link)

    def other(self, slot: str | None) -> str:
        return "b" if slot == "a" else "a"

    def inactive(self) -> str:
        """Slot to install into: neither current nor committed."""
        cur = self.current()
        com = self.committed()
        for s in SLOTS:
            if s != cur and s != com:
                return s
        raise SlotError("no inactive slot (current and committed differ: update in flight?)")

    def switch_current(self, slot: str) -> None:
        self.slot_path(slot)
        atomic_symlink(f"slots/{slot}", self.current_link)

    def switch_committed(self, slot: str) -> None:
        self.slot_path(slot)
        atomic_symlink(f"slots/{slot}", self.committed_link)

    def wipe(self, slot: str) -> None:
        p = self.slot_path(slot)
        if p.exists() or p.is_symlink():
            rmtree(p)

    def release_info(self, slot: str | None) -> dict[str, Any] | None:
        if slot is None:
            return None
        p = self.slot_path(slot) / "release.json"
        try:
            return json.loads(p.read_text())
        except (OSError, ValueError):
            return None

    def version(self, slot: str | None) -> str | None:
        info = self.release_info(slot)
        v = info.get("version") if info else None
        return v if isinstance(v, str) else None
