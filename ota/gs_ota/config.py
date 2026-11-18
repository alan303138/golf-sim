"""Agent configuration (``/etc/golfsim/ota.toml`` on the device)."""
from __future__ import annotations

import tomllib
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

DEFAULT_CONFIG_PATH = Path("/etc/golfsim/ota.toml")


@dataclass
class AgentConfig:
    # Where to fetch the manifest. "{channel}" is substituted.
    manifest_url: str = "https://updates.invalid/golfsim/{channel}/manifest.json"
    channel: str = "stable"
    # Ed25519 public keys (PEM). Any one valid signature is accepted (key rotation).
    trusted_keys: list[str] = field(default_factory=lambda: ["/etc/golfsim/ota/release-ed25519.pub.pem"])

    # Layout: <root>/slots/{a,b}, <root>/current, <root>/committed
    root: str = "/opt/golfsim"
    # Agent-private state; must survive reboots.
    state_dir: str = "/var/lib/gs-ota"

    # Health check of the restarted service.
    status_url: str = "http://127.0.0.1:8080/api/v1/status"
    health_timeout_s: float = 120.0
    health_interval_s: float = 2.0
    health_successes: int = 2           # consecutive good probes required
    max_boot_attempts: int = 3          # agent restarts during an unconfirmed update
    smoke_timeout_s: float = 120.0
    device_python: str = "/usr/bin/python3"

    # systemd integration: "path-unit" (unprivileged agent, see deploy/),
    # "systemctl" (agent runs as root) or "none" (tests/dev).
    service_control: str = "path-unit"
    service_unit: str = "gs-service.service"
    ctl_dir: str = "/run/gs-ota/ctl"
    service_timeout_s: float = 60.0

    # Control socket used by gs_service (gs_ota.client.AgentUpdater).
    socket: str = "/run/gs-ota/agent.sock"

    # Periodic check; 0 disables. auto_apply installs without user action.
    check_interval_s: float = 6 * 3600
    auto_apply: bool = False

    # Firmware: "module:callable" returning an *unopened* gs_common McuLink.
    mcu_link_factory: str = "gs_mcu:open_link"
    mcu_link_kwargs: dict[str, Any] = field(default_factory=dict)
    uf2_drive_globs: list[str] = field(default_factory=lambda: [
        "/run/gs-ota/rpi-rp2", "/media/*/RPI-RP2", "/run/media/*/RPI-RP2", "/mnt/RPI-RP2"])
    picotool: str = "auto"             # "auto" (use if on PATH), "never", or a path
    bootloader_timeout_s: float = 30.0
    reenumerate_timeout_s: float = 30.0
    flash_attempts: int = 3
    update_firmware: bool = True

    http_timeout_s: float = 30.0
    trust_env_proxy: bool = True
    max_unpacked_bytes: int = 4 * 1024 ** 3
    unpack_ratio: float = 4.0           # free space needed in slots = size * ratio

    @property
    def resolved_manifest_url(self) -> str:
        return self.manifest_url.replace("{channel}", self.channel)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "AgentConfig":
        known = {f.name for f in fields(cls)}
        unknown = set(d) - known
        if unknown:
            raise ValueError(f"unknown config keys: {sorted(unknown)}")
        return cls(**d)

    @classmethod
    def load(cls, path: str | Path = DEFAULT_CONFIG_PATH) -> "AgentConfig":
        with open(path, "rb") as f:
            return cls.from_dict(tomllib.load(f))
