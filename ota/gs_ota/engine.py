"""Update transaction engine (synchronous; runs inside the OTA agent).

Transaction phases persisted in state.json (``txn.phase``) and what a restart
of the agent does when it finds them (``recover()``):

  download  artifacts being fetched          -> discard staged slot, keep .part
  stage     sensor bundle being unpacked     -> discard staged slot
  staged    unpacked + smoke-tested          -> discard staged slot
  firmware  MCU being flashed                -> re-flash cached verified .uf2
                                                (ROM bootloader: always safe),
                                                discard staged slot
  switch    about to flip `current`          -> if flipped: as 'health',
                                                else discard staged slot
  health    `current` -> new slot, service   -> boot_attempts += 1; if more than
            restarted, waiting for health       max_boot_attempts: roll back,
                                                else restart + health check again
  rollback  flipping back                    -> finish the rollback

Order inside apply(): download all -> verify -> stage sensor into the inactive
slot -> stop service -> flash firmware -> switch `current` -> restart ->
health -> commit (`committed` -> new slot). Firmware goes first so a new
sensor release may rely on new firmware; the firmware must stay compatible
with the previous sensor release so a sensor rollback keeps working.
"""
from __future__ import annotations

import logging
import re
import shutil
import subprocess
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable
from urllib.parse import urlparse

import httpx

from gs_common.interfaces import UpdateState

from . import keys as _keys
from .config import AgentConfig
from .download import (DownloadError, IntegrityError, check_free_space, download_artifact,
                       fetch_bytes, make_client, verify_file)
from .manifest import (MAX_MANIFEST_BYTES, Artifact, Manifest, ManifestError, parse_released,
                       resolve_url, verify_manifest)
from .mcu import FirmwareError, FirmwareFlasher, validate_uf2
from .semver import Version
from .services import (HealthProbe, HttpHealthProbe, NullServiceController, ServiceController,
                       make_service_controller, reported_version, wait_healthy)
from .slots import SlotManager
from .state import StateStore
from .tarsafe import UnsafeArchiveError, safe_extract

log = logging.getLogger(__name__)

SENSOR = "sensor"
FIRMWARE = "firmware"
MAX_SIG_BYTES = 4096

SmokeRunner = Callable[[list[str], float], tuple[int, str]]


class UpdateError(Exception):
    pass


def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def default_smoke_runner(cmd: list[str], timeout: float) -> tuple[int, str]:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        return 127, str(e)
    return r.returncode, (r.stderr or r.stdout)[-2000:]


def _local_name(art: Artifact) -> str:
    base = Path(urlparse(art.url).path).name
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+-]{0,127}", base):
        base = f"{art.name}-{art.version}.bin"
    return base


class UpdateEngine:
    def __init__(self, config: AgentConfig, *,
                 service: ServiceController | None = None,
                 health_probe: HealthProbe | None = None,
                 flasher: FirmwareFlasher | None | bool = True,
                 http_client: httpx.Client | None = None,
                 smoke_runner: SmokeRunner = default_smoke_runner,
                 fw_version_source: Callable[[], str | None] | None = None,
                 trusted_keys: list | None = None,
                 sleep: Callable[[float], None] = time.sleep,
                 on_committed: Callable[[str], None] | None = None):
        self.cfg = config
        self.slots = SlotManager(config.root)
        self.state_dir = Path(config.state_dir)
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self.downloads = self.state_dir / "downloads"
        self.fw_cache = self.state_dir / "firmware"
        self.store = StateStore(self.state_dir / "state.json")
        self.service = service or make_service_controller(
            config.service_control, config.service_unit, config.ctl_dir, config.service_timeout_s)
        self.health_probe = health_probe or HttpHealthProbe(config.status_url)
        self.http = http_client or make_client(config.http_timeout_s, config.trust_env_proxy)
        self.smoke_runner = smoke_runner
        self.sleep = sleep
        self.on_committed = on_committed
        self._trusted = trusted_keys
        if flasher is True:
            flasher = self._default_flasher()
        self.flasher: FirmwareFlasher | None = flasher or None
        self.fw_version_source = fw_version_source or self._fw_version_from_status
        self._state = UpdateState()
        self._callbacks: list[Callable[[UpdateState], None]] = []
        self._lock = threading.Lock()          # one check/apply at a time
        self._manifest: Manifest | None = None
        self._refresh_current()

    # ------------------------------------------------------------ plumbing
    def _default_flasher(self) -> FirmwareFlasher | None:
        if not self.cfg.update_firmware:
            return None
        from .mcu import load_entry
        try:
            factory = load_entry(self.cfg.mcu_link_factory)
        except (ImportError, AttributeError, ValueError) as e:
            log.warning("MCU link factory %r unavailable (%s); firmware updates disabled",
                        self.cfg.mcu_link_factory, e)
            return None
        kwargs = dict(self.cfg.mcu_link_kwargs)
        return FirmwareFlasher(lambda: factory(**kwargs), self.cfg.uf2_drive_globs,
                               self.cfg.picotool, self.cfg.bootloader_timeout_s,
                               self.cfg.reenumerate_timeout_s, self.cfg.flash_attempts)

    def trusted_keys(self) -> list:
        if self._trusted is None:
            self._trusted = _keys.load_ed25519_public_keys(self.cfg.trusted_keys)
        return self._trusted

    def on_change(self, cb: Callable[[UpdateState], None]) -> None:
        self._callbacks.append(cb)

    def state(self) -> UpdateState:
        s = self._state
        return UpdateState(s.phase, s.progress, s.message, dict(s.current),
                           dict(s.available), s.notes)

    def _set(self, phase: str | None = None, progress: float | None = None,
             message: str | None = None) -> None:
        if phase is not None:
            self._state.phase = phase
        if progress is not None:
            self._state.progress = max(0.0, min(1.0, progress))
        if message is not None:
            self._state.message = message
        snap = self.state()
        for cb in list(self._callbacks):
            try:
                cb(snap)
            except Exception:  # noqa: BLE001 - never let a listener break an update
                log.exception("update listener failed")

    def _fw_version_from_status(self) -> str | None:
        try:
            r = httpx.get(self.cfg.status_url, timeout=3.0, trust_env=False)
            doc = r.json() if r.status_code == 200 else {}
        except (httpx.HTTPError, ValueError):
            doc = {}
        v = doc.get("fw_version") if isinstance(doc, dict) else None
        if not isinstance(v, str) and isinstance(doc, dict) and isinstance(doc.get("versions"), dict):
            v = doc["versions"].get("firmware")
        return v if isinstance(v, str) else None

    def installed_versions(self) -> dict[str, str]:
        out: dict[str, str] = {}
        sv = self.slots.version(self.slots.committed()) or self.store.data["installed"].get(SENSOR)
        if sv:
            out[SENSOR] = sv
        fv = None
        try:
            fv = self.fw_version_source()
        except Exception:  # noqa: BLE001
            fv = None
        fv = fv or self.store.data["installed"].get(FIRMWARE)
        if fv:
            out[FIRMWARE] = fv
        return out

    def _refresh_current(self) -> None:
        self._state.current = self.installed_versions()

    # --------------------------------------------------------------- check
    def check(self) -> UpdateState:
        with self._lock:
            try:
                self._check_locked()
            except (DownloadError, ManifestError, UpdateError, _keys.KeyMaterialError, OSError) as e:
                self._manifest = None
                self._state.available = {}
                self._set("failed", 0.0, f"update check failed: {e}")
            return self.state()

    def _check_locked(self) -> Manifest:
        self._set("checking", 0.0, "checking for updates")
        url = self.cfg.resolved_manifest_url
        data = fetch_bytes(self.http, url, MAX_MANIFEST_BYTES)
        sig = fetch_bytes(self.http, url + ".sig", MAX_SIG_BYTES)
        m = verify_manifest(data, sig, self.trusted_keys())   # nothing trusted before this
        if m.channel != self.cfg.channel:
            raise UpdateError(f"manifest is for channel {m.channel!r}, expected {self.cfg.channel!r}")
        # Anti-replay: never accept a manifest older than one already seen.
        hr = self.store.data["highest_released"].get(m.channel)
        if hr and m.released_dt < parse_released(hr):
            raise UpdateError(f"stale manifest (released {m.released}, already saw {hr}); "
                              "possible replay/rollback attack")
        hs = dict(self.store.data["highest_seen"])
        for name, a in m.artifacts.items():
            if name not in hs or Version.parse(hs[name]) < a.semver:
                hs[name] = a.version
        self.store.update(highest_released={**self.store.data["highest_released"],
                                            m.channel: m.released}, highest_seen=hs)
        self._manifest = m
        self._refresh_current()
        available, notes = self._plan(m)
        self._state.available = {k: a.version for k, a in available.items()}
        self._state.notes = m.notes
        if available:
            self._set("available", 0.0, "update available: " + ", ".join(
                f"{k} {a.version}" for k, a in available.items()))
        else:
            self._set("up_to_date", 0.0, "; ".join(notes) or "up to date")
        return m

    def _plan(self, m: Manifest) -> tuple[dict[str, Artifact], list[str]]:
        """Artifacts that are newer than installed and allowed by min_version.
        Lower versions are refused (no downgrades)."""
        cur = self._state.current
        out: dict[str, Artifact] = {}
        notes: list[str] = []
        for name in (FIRMWARE, SENSOR):
            a = m.artifacts.get(name)
            if a is None:
                continue
            if name == FIRMWARE and self.flasher is None:
                notes.append("firmware updates disabled (no MCU link)")
                continue
            inst = cur.get(name)
            if inst is not None:
                iv = Version.parse(inst)
                if a.semver < iv:
                    notes.append(f"refusing {name} downgrade {inst} -> {a.version}")
                    continue
                if a.semver == iv:
                    continue
                if a.min_version and iv < Version.parse(a.min_version):
                    notes.append(f"{name} {a.version} requires installed >= {a.min_version} "
                                 f"(have {inst}); install an intermediate release first")
                    continue
            elif name == SENSOR and a.min_version:
                notes.append(f"sensor {a.version} requires an installed version >= {a.min_version}")
                continue
            out[name] = a
        return out, notes

    # --------------------------------------------------------------- apply
    def apply(self, auto: bool = False) -> bool:
        """Run a full update transaction. Returns True if something was
        installed and confirmed. Never raises for update failures."""
        if not self._lock.acquire(blocking=False):
            raise UpdateError("an update operation is already running")
        try:
            if self.store.txn:
                raise UpdateError("an interrupted update must be recovered first")
            m = self._check_locked()
            plan, _ = self._plan(m)
            if auto:
                failed = self.store.data["failed_versions"]
                plan = {k: a for k, a in plan.items() if a.version not in failed.get(k, [])}
            if not plan:
                self._set("up_to_date", 1.0, self._state.message)
                return False
            return self._run_txn(m, plan)
        except (DownloadError, ManifestError, UpdateError, _keys.KeyMaterialError) as e:
            self._fail(str(e))
            return False
        except Exception as e:  # noqa: BLE001
            log.exception("unexpected error during update")
            self._fail(f"internal error: {e}")
            return False
        finally:
            self._lock.release()

    def _fail(self, msg: str) -> None:
        self.store.update(last_result={"ok": False, "message": msg, "at": _now()})
        self._refresh_current()
        self._set("failed", 0.0, msg)

    def _run_txn(self, m: Manifest, plan: dict[str, Artifact]) -> bool:
        url = self.cfg.resolved_manifest_url
        txn: dict[str, Any] = {"id": uuid.uuid4().hex, "phase": "download", "started": _now(),
                               "manifest": m.digest, "boot_attempts": 0, "artifacts": {}}
        for name, a in plan.items():
            txn["artifacts"][name] = {
                "version": a.version, "from": self._state.current.get(name),
                "url": resolve_url(url, a.url), "sha256": a.sha256, "size": a.size,
                "file": str(self.downloads / _local_name(a))}
        if SENSOR in plan:
            txn["target_slot"] = self.slots.inactive()
            txn["previous_slot"] = self.slots.current()
        self.store.set_txn(txn)

        # 1. download --------------------------------------------------------
        total = sum(a.size for a in plan.values())
        done_before = 0
        for name, info in txn["artifacts"].items():
            self._set("downloading", done_before / total, f"downloading {name} {info['version']}")

            def prog(d: int, _t: int, base: int = done_before) -> None:
                self._state.progress = (base + d) / total
                if d == _t or d % (1 << 20) < 65536:
                    self._set()
            try:
                download_artifact(self.http, info["url"], Path(info["file"]), info["sha256"],
                                  info["size"], progress=prog)
            except DownloadError:
                self._abort_pre_switch()
                raise
            done_before += info["size"]

        # 2. verify (again, right before use) --------------------------------
        self._set("verifying", 0.0, "verifying downloads")
        try:
            for name, info in txn["artifacts"].items():
                verify_file(Path(info["file"]), info["sha256"], info["size"])
                if name == FIRMWARE:
                    validate_uf2(Path(info["file"]).read_bytes())
        except (IntegrityError, FirmwareError) as e:
            Path(txn["artifacts"][name]["file"]).unlink(missing_ok=True)
            self._abort_pre_switch()
            raise UpdateError(str(e)) from e

        # 3. stage sensor into the inactive slot -----------------------------
        if SENSOR in plan:
            self.store.txn_update(phase="stage")
            self._set("installing", 0.1, "unpacking sensor software")
            try:
                self._stage_sensor(txn["target_slot"], txn["artifacts"][SENSOR])
            except (UpdateError, UnsafeArchiveError, OSError) as e:
                self._abort_pre_switch()
                raise UpdateError(f"sensor install failed: {e}") from e
            self.store.txn_update(phase="staged")

        # 4. firmware ---------------------------------------------------------
        if FIRMWARE in plan:
            self.store.txn_update(phase="firmware")
            info = txn["artifacts"][FIRMWARE]
            self._set("installing", 0.4, "stopping sensor service for firmware update")
            try:
                self.service.stop()
                self._flash(Path(info["file"]), info["version"])
            except Exception as e:  # noqa: BLE001
                self._abort_pre_switch(start_service=True)
                raise UpdateError(f"firmware update failed: {e}") from e
            if SENSOR not in plan:
                self.service.start()
                self._finish_ok(f"firmware updated to {info['version']}")
                return True

        # 5. switch + health ----------------------------------------------------
        info = txn["artifacts"][SENSOR]
        self.store.txn_update(phase="switch")
        self.slots.switch_current(txn["target_slot"])
        self.store.txn_update(phase="health", boot_attempts=0)
        return self._restart_and_confirm()

    def _flash(self, uf2: Path, version: str) -> None:
        assert self.flasher is not None

        def prog(frac: float, msg: str) -> None:
            self._set("installing", 0.4 + 0.3 * frac, msg)
        verified = self.flasher.flash(uf2, version, progress=prog)
        self.fw_cache.mkdir(parents=True, exist_ok=True)
        shutil.copy2(uf2, self.fw_cache / f"timing-mcu-{verified}.uf2")
        self.store.update(installed={**self.store.data["installed"], FIRMWARE: verified})

    def _stage_sensor(self, slot: str, info: dict[str, Any]) -> None:
        self.slots.wipe(slot)
        dest = self.slots.slot_path(slot)
        check_free_space(self.slots.slots_dir, int(info["size"] * self.cfg.unpack_ratio))
        safe_extract(Path(info["file"]), dest, max_bytes=self.cfg.max_unpacked_bytes)
        rel = self.slots.release_info(slot)
        if not rel:
            raise UpdateError("bundle has no valid release.json")
        if rel.get("version") != info["version"]:
            raise UpdateError(f"bundle version {rel.get('version')!r} != manifest {info['version']!r}")
        launcher = dest / "bin" / "gs-python"
        if not launcher.is_file():
            raise UpdateError("bundle has no bin/gs-python launcher")
        mods = [m for m in rel.get("smoke_imports", []) if re.fullmatch(r"[A-Za-z_][\w.]*", m)]
        want_py = rel.get("python")
        code = "import sys\n"
        if isinstance(want_py, str) and re.fullmatch(r"\d+\.\d+", want_py):
            code += (f"v='%d.%d'%sys.version_info[:2]\n"
                     f"assert v=={want_py!r}, 'bundle built for Python {want_py}, device has '+v\n")
        for mod in mods:
            code += f"import {mod}\n"
        # Pre-compile bytecode while the slot is still writable by the agent
        # (the service user cannot write into slots). Failure is not fatal.
        self._set("installing", 0.2, "compiling bytecode")
        self.smoke_runner([str(launcher), "-m", "compileall", "-q", str(dest / "lib")],
                          self.cfg.smoke_timeout_s)
        self._set("installing", 0.3, "smoke-testing new release")
        rc, out = self.smoke_runner([str(launcher), "-c", code], self.cfg.smoke_timeout_s)
        if rc != 0:
            raise UpdateError(f"smoke test failed (rc={rc}): {out.strip()[-500:]}")

    def _abort_pre_switch(self, start_service: bool = False) -> None:
        """Undo everything before `current` was switched."""
        txn = self.store.txn
        if txn and txn.get("target_slot"):
            slot = txn["target_slot"]
            if slot not in (self.slots.current(), self.slots.committed()):
                self.slots.wipe(slot)
        self.store.set_txn(None)
        if start_service:
            try:
                self.service.start()
            except Exception:  # noqa: BLE001
                log.exception("could not start service after aborted update")

    def _restart_and_confirm(self) -> bool:
        txn = self.store.txn
        assert txn is not None
        version = txn["artifacts"][SENSOR]["version"]
        self._set("rebooting", 0.0, f"restarting service on {version}")
        try:
            self.service.restart()
        except Exception as e:  # noqa: BLE001
            return self._rollback(f"service restart failed: {e}")

        def tick(frac: float, detail: str) -> None:
            self._set("rebooting", frac, f"waiting for {version} to report healthy: {detail}")
        ok, detail = wait_healthy(self.health_probe, version, self.cfg.health_timeout_s,
                                  self.cfg.health_interval_s, self.cfg.health_successes,
                                  sleep=self.sleep, on_tick=tick)
        if not ok:
            return self._rollback(detail)
        self._commit()
        return True

    def _commit(self) -> None:
        txn = self.store.txn
        assert txn is not None
        info = txn["artifacts"][SENSOR]
        self.slots.switch_committed(txn["target_slot"])
        self.store.update(installed={**self.store.data["installed"], SENSOR: info["version"]})
        Path(info["file"]).unlink(missing_ok=True)
        self._finish_ok(f"updated sensor software to {info['version']}")
        if self.on_committed:
            self.on_committed(info["version"])

    def _finish_ok(self, msg: str) -> None:
        self.store.update(txn=None, last_result={"ok": True, "message": msg, "at": _now()})
        self._refresh_current()
        self._state.available = {}
        self._set("up_to_date", 1.0, msg)

    def _rollback(self, reason: str) -> bool:
        txn = self.store.txn
        assert txn is not None
        self.store.txn_update(phase="rollback", rollback_reason=reason)
        prev = txn.get("previous_slot") or self.slots.committed()
        log.error("rolling back sensor update: %s", reason)
        self._set("failed", 0.0, f"update failed, rolling back: {reason}")
        if prev:
            self.slots.switch_current(prev)
        try:
            self.service.restart()
        except Exception:  # noqa: BLE001
            log.exception("service restart after rollback failed")
        info = txn["artifacts"].get(SENSOR, {})
        fv = dict(self.store.data["failed_versions"])
        if info.get("version"):
            fv[SENSOR] = sorted(set(fv.get(SENSOR, [])) | {info["version"]})
        msg = (f"sensor update to {info.get('version')} rolled back to "
               f"{self.slots.version(prev)}: {reason}")
        self.store.update(txn=None, failed_versions=fv,
                          last_result={"ok": False, "message": msg, "at": _now()})
        self._refresh_current()
        self._set("failed", 0.0, msg)
        return False

    # ------------------------------------------------------------- recovery
    def recover(self) -> str:
        """Call once at agent start, before serving requests. Returns a short
        description of what was done."""
        with self._lock:
            txn = self.store.txn
            if txn is None:
                cur, com = self.slots.current(), self.slots.committed()
                if com and cur != com:
                    log.warning("current (%s) != committed (%s) without a transaction; "
                                "reverting to committed", cur, com)
                    self.slots.switch_current(com)
                    self._safe(self.service.restart)
                    return "reverted current to committed slot"
                return "nothing to recover"
            phase = txn["phase"]
            log.warning("recovering interrupted update (phase %s)", phase)
            if phase in ("download", "stage", "staged"):
                self._abort_pre_switch(start_service=True)
                self._fail_quiet(f"update interrupted during {phase}; rolled back cleanly")
                return "aborted pre-switch"
            if phase == "firmware":
                info = txn["artifacts"][FIRMWARE]
                msg = "firmware update interrupted"
                try:
                    verify_file(Path(info["file"]), info["sha256"], info["size"])
                    if self.flasher is None:
                        raise FirmwareError("no MCU link")
                    self._flash(Path(info["file"]), info["version"])
                    msg += f"; re-flashed {info['version']} successfully"
                except Exception as e:  # noqa: BLE001
                    msg += f"; re-flash failed ({e}); retry the update"
                self._abort_pre_switch(start_service=True)
                self._fail_quiet(msg + "; sensor software unchanged")
                return "firmware recovered"
            if phase == "switch":
                if self.slots.current() != txn.get("target_slot"):
                    self._abort_pre_switch(start_service=True)
                    self._fail_quiet("update interrupted before switching; rolled back cleanly")
                    return "aborted pre-switch"
                self.store.txn_update(phase="health")
                phase = "health"
            if phase == "health":
                attempts = int(txn.get("boot_attempts", 0)) + 1
                self.store.txn_update(boot_attempts=attempts)
                if attempts > self.cfg.max_boot_attempts:
                    self._rollback(f"new release did not confirm health after {attempts - 1} restarts")
                    return "rolled back (boot attempts)"
                ok = self._restart_and_confirm()
                return "committed" if ok else "rolled back (health)"
            if phase == "rollback":
                self._rollback(txn.get("rollback_reason", "interrupted rollback"))
                return "rollback completed"
            log.error("unknown transaction phase %r; rolling back", phase)
            self._rollback(f"unknown phase {phase}")
            return "rolled back (unknown phase)"

    def _fail_quiet(self, msg: str) -> None:
        self.store.update(last_result={"ok": False, "message": msg, "at": _now()})
        self._refresh_current()
        self._set("failed", 0.0, msg)

    @staticmethod
    def _safe(fn: Callable[[], None]) -> None:
        try:
            fn()
        except Exception:  # noqa: BLE001
            log.exception("hook failed")

    # ------------------------------------------------------------ bootstrap
    def bootstrap(self, manifest_path: Path, bundle: Path | None = None) -> str:
        """First install on a fresh device from a local, signed release
        directory (manifest.json + .sig + sensor bundle). Installs into slot
        'a' and points both symlinks at it. Refuses if already installed."""
        with self._lock:
            if self.slots.committed():
                raise UpdateError("device already has a committed release")
            d = Path(manifest_path)
            data = d.read_bytes()
            sig = d.with_name(d.name + ".sig").read_bytes()
            m = verify_manifest(data, sig, self.trusted_keys())
            a = m.artifacts.get(SENSOR)
            if a is None:
                raise UpdateError("manifest has no sensor artifact")
            src = Path(bundle) if bundle else d.parent / Path(urlparse(a.url).path).name
            verify_file(src, a.sha256, a.size)
            self.slots.ensure_layout()
            info = {"version": a.version, "file": str(src), "size": a.size}
            self._stage_sensor("a", info)
            self.slots.switch_current("a")
            self.slots.switch_committed("a")
            self.store.update(installed={**self.store.data["installed"], SENSOR: a.version},
                              highest_released={m.channel: m.released})
            self._refresh_current()
            return a.version
