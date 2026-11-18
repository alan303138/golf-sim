"""User configuration persisted as a JSON file."""
from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field


class UserConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    distance_unit: Literal["yd", "m"] = "yd"
    speed_unit: Literal["mph", "kmh", "ms"] = "mph"
    handedness: Literal["right", "left"] = "right"
    altitude_m: float = Field(0.0, ge=-500, le=5000)
    placement_distance_m: float = Field(2.0, ge=0.5, le=6.0)
    gspro_enabled: bool = False
    gspro_host: str = "127.0.0.1"
    gspro_port: int = Field(921, ge=1, le=65535)


class ConfigStore:
    def __init__(self, path: Path, defaults: dict[str, Any] | None = None):
        self.path = path
        self._lock = threading.Lock()
        base = UserConfig(**(defaults or {}))
        self._cfg = base
        if path.exists():
            try:
                raw = json.loads(path.read_text())
                merged = {**base.model_dump(), **{k: v for k, v in raw.items() if k in UserConfig.model_fields}}
                self._cfg = UserConfig(**merged)
            except Exception:  # corrupt file: fall back to defaults, keep the bad file aside
                try:
                    path.rename(path.with_suffix(".json.bad"))
                except OSError:
                    pass

    def get(self) -> dict[str, Any]:
        with self._lock:
            return self._cfg.model_dump()

    def update(self, patch: dict[str, Any]) -> dict[str, Any]:
        """Merge a partial update. Raises pydantic.ValidationError on bad input."""
        with self._lock:
            merged = {**self._cfg.model_dump(), **patch}
            new = UserConfig(**merged)
            self._cfg = new
            self._save(new)
            return new.model_dump()

    def _save(self, cfg: UserConfig) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(cfg.model_dump(), indent=2))
        os.replace(tmp, self.path)
