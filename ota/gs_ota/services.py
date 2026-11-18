"""Service control and health-check hooks (injectable for tests)."""
from __future__ import annotations

import json
import logging
import os
import subprocess
import time
import uuid
from pathlib import Path
from typing import Callable, Protocol

import httpx

from .fsutil import atomic_write_bytes

log = logging.getLogger(__name__)


class ServiceError(Exception):
    pass


class ServiceController(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def restart(self) -> None: ...


class NullServiceController:
    """No-op (tests / development without systemd)."""

    def __init__(self) -> None:
        self.calls: list[str] = []

    def start(self) -> None:
        self.calls.append("start")

    def stop(self) -> None:
        self.calls.append("stop")

    def restart(self) -> None:
        self.calls.append("restart")


class CallbackServiceController:
    def __init__(self, start: Callable[[], None], stop: Callable[[], None],
                 restart: Callable[[], None] | None = None):
        self._start, self._stop = start, stop
        self._restart = restart

    def start(self) -> None:
        self._start()

    def stop(self) -> None:
        self._stop()

    def restart(self) -> None:
        if self._restart:
            self._restart()
        else:
            self._stop()
            self._start()


def _is_active(unit: str) -> str:
    # `systemctl is-active` needs no privileges.
    r = subprocess.run(["systemctl", "is-active", unit], capture_output=True, text=True)
    return r.stdout.strip()


class SystemctlController:
    """Direct systemctl calls; requires the agent to run as root."""

    def __init__(self, unit: str, timeout: float = 60.0):
        self.unit, self.timeout = unit, timeout

    def _run(self, action: str) -> None:
        r = subprocess.run(["systemctl", action, self.unit], capture_output=True, text=True,
                           timeout=self.timeout)
        if r.returncode != 0:
            raise ServiceError(f"systemctl {action} {self.unit}: {r.stderr.strip()}")

    def start(self) -> None:
        self._run("start")

    def stop(self) -> None:
        self._run("stop")

    def restart(self) -> None:
        self._run("restart")


class PathUnitController:
    """Unprivileged service control without sudo or polkit.

    The agent drops a request file (``start``/``stop``/``restart``) into
    ``ctl_dir/requests``; ``gs-ota-svcctl.path`` (DirectoryNotEmpty=) activates
    the root-owned oneshot ``gs-ota-svcctl.service``, which runs the
    whitelisted ``systemctl <action> gs-service.service`` and deletes the
    request. The agent then polls ``systemctl is-active`` (unprivileged).
    See ota/deploy/.
    """

    def __init__(self, ctl_dir: str | Path, unit: str, timeout: float = 60.0,
                 is_active: Callable[[str], str] = _is_active):
        self.req_dir = Path(ctl_dir) / "requests"
        self.unit, self.timeout = unit, timeout
        self.is_active = is_active

    def _request(self, action: str, want: set[str]) -> None:
        self.req_dir.mkdir(parents=True, exist_ok=True)
        name = f"{int(time.time() * 1000)}-{uuid.uuid4().hex[:8]}"
        atomic_write_bytes(self.req_dir / name, (action + "\n").encode())
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if not (self.req_dir / name).exists() and self.is_active(self.unit) in want:
                return
            time.sleep(0.5)
        raise ServiceError(f"{action} {self.unit}: not confirmed within {self.timeout}s "
                           "(is gs-ota-svcctl.path enabled?)")

    def start(self) -> None:
        self._request("start", {"active", "activating"})

    def stop(self) -> None:
        self._request("stop", {"inactive", "failed"})

    def restart(self) -> None:
        self._request("restart", {"active", "activating"})


def make_service_controller(kind: str, unit: str, ctl_dir: str, timeout: float) -> ServiceController:
    if kind == "systemctl":
        return SystemctlController(unit, timeout)
    if kind == "path-unit":
        return PathUnitController(ctl_dir, unit, timeout)
    if kind == "none":
        return NullServiceController()
    raise ValueError(f"unknown service_control {kind!r}")


# --------------------------------------------------------------- health check
HealthProbe = Callable[[str], tuple[bool, str]]  # expected sensor version -> (ok, detail)


def reported_version(status: dict) -> str | None:
    """Software version from GET /api/v1/status. Accepts the shapes a status
    document may reasonably have: sw_version (as in the WS hello message),
    versions.sensor / versions.sw, or version."""
    for key in ("sw_version", "version"):
        v = status.get(key)
        if isinstance(v, str):
            return v
    vers = status.get("versions")
    if isinstance(vers, dict):
        for key in ("sensor", "sw", "sw_version", "software"):
            v = vers.get(key)
            if isinstance(v, str):
                return v
    return None


class HttpHealthProbe:
    """One probe of the local service: HTTP 200, JSON, reported version equal
    to the version just installed. Device state 'error' is tolerated (e.g. a
    camera unplugged) because that is not caused by the software update."""

    def __init__(self, url: str, timeout: float = 3.0):
        self.url = url
        self.client = httpx.Client(timeout=timeout, trust_env=False)

    def __call__(self, expected: str) -> tuple[bool, str]:
        try:
            r = self.client.get(self.url)
        except httpx.HTTPError as e:
            return False, f"status endpoint unreachable: {e.__class__.__name__}"
        if r.status_code != 200:
            return False, f"status endpoint HTTP {r.status_code}"
        try:
            doc = r.json()
        except (ValueError, json.JSONDecodeError):
            return False, "status endpoint returned non-JSON"
        if not isinstance(doc, dict):
            return False, "status endpoint returned non-object"
        v = reported_version(doc)
        if v != expected:
            return False, f"service reports version {v!r}, expected {expected!r}"
        return True, "healthy"


def wait_healthy(probe: HealthProbe, expected: str, timeout: float, interval: float,
                 successes: int = 2, sleep: Callable[[float], None] = time.sleep,
                 on_tick: Callable[[float, str], None] | None = None) -> tuple[bool, str]:
    start = time.monotonic()
    good = 0
    detail = "no probe yet"
    while True:
        ok, detail = probe(expected)
        good = good + 1 if ok else 0
        if good >= successes:
            return True, detail
        elapsed = time.monotonic() - start
        if on_tick:
            on_tick(min(1.0, elapsed / timeout) if timeout else 1.0, detail)
        if elapsed >= timeout:
            return False, f"health check timed out after {timeout:.0f}s: {detail}"
        sleep(interval)


def env_python() -> str:
    return os.environ.get("GS_PYTHON", "/usr/bin/python3")
