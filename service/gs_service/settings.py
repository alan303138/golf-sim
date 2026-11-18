"""Process-level settings (CLI / environment). User settings live in config.py."""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path


def default_web_dir() -> Path:
    env = os.environ.get("GS_WEB_DIR")
    if env:
        return Path(env)
    # service/gs_service/settings.py -> service/web
    return Path(__file__).resolve().parent.parent / "web"


@dataclass
class Settings:
    host: str = "0.0.0.0"
    port: int = 8080
    data_dir: Path = field(default_factory=lambda: Path(os.environ.get("GS_DATA_DIR", "./gs_data")))
    source: str = "mock"          # "mock" or "module:Attr" entry point
    updater: str = "mock"         # "mock" or "module:Attr" entry point
    web_dir: Path = field(default_factory=default_web_dir)
    mdns: bool = True
    device_id: str | None = None  # default: persisted random id in data_dir
    # GSPro adapter defaults (also changeable at runtime through /api/v1/config)
    gspro_enabled: bool = False
    gspro_host: str = "127.0.0.1"
    gspro_port: int = 921

    @classmethod
    def from_env(cls) -> "Settings":
        e = os.environ
        s = cls()
        s.host = e.get("GS_HOST", s.host)
        s.port = int(e.get("GS_PORT", s.port))
        s.source = e.get("GS_SOURCE", s.source)
        s.updater = e.get("GS_UPDATER", s.updater)
        s.mdns = e.get("GS_MDNS", "1") not in ("0", "false", "no")
        s.device_id = e.get("GS_DEVICE_ID") or None
        s.gspro_enabled = e.get("GS_GSPRO", "0") in ("1", "true", "yes")
        s.gspro_host = e.get("GS_GSPRO_HOST", s.gspro_host)
        s.gspro_port = int(e.get("GS_GSPRO_PORT", s.gspro_port))
        return s
