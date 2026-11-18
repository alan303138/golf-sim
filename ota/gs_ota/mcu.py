"""RP2040 timing-MCU firmware update through the ROM bootloader.

The RP2040 boot ROM cannot be overwritten, so a failed or interrupted flash
always leaves the chip in (or able to enter) BOOTSEL mode: retrying is safe.

Flow: McuLink.reboot_to_bootloader() -> wait for the RPI-RP2 mass-storage
drive (or `picotool info`) -> copy the .uf2 (or `picotool load -x`) -> wait
for the application to re-enumerate -> McuLink.info().fw_version must equal
the manifest version.

gs_mcu is never imported directly: the link comes from a factory given as an
entry-point string ``"module:callable"`` (config ``mcu_link_factory``).
"""
from __future__ import annotations

import glob
import importlib
import logging
import os
import shutil
import struct
import subprocess
import time
from pathlib import Path
from typing import Any, Callable

from gs_common.interfaces import McuLink

log = logging.getLogger(__name__)

UF2_MAGIC0 = 0x0A324655
UF2_MAGIC1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_FLAG_FAMILY = 0x00002000
UF2_FLAG_NOT_MAIN_FLASH = 0x00000001
RP2040_FAMILY = 0xE48BFF56
RP2040_FLASH_START = 0x10000000
RP2040_FLASH_END = 0x11000000


class FirmwareError(Exception):
    pass


def load_entry(spec: str) -> Callable[..., Any]:
    """Resolve "package.module:attr.sub" to an object."""
    if ":" not in spec:
        raise ValueError(f"entry point must look like 'module:callable', got {spec!r}")
    mod_name, attr = spec.split(":", 1)
    obj: Any = importlib.import_module(mod_name)
    for part in attr.split("."):
        obj = getattr(obj, part)
    return obj


def validate_uf2(data: bytes, families: tuple[int, ...] = (RP2040_FAMILY,)) -> int:
    """Structural check of a UF2 image for the RP2040. Returns block count."""
    if not data or len(data) % 512:
        raise FirmwareError("UF2 size is not a multiple of 512")
    nblocks = len(data) // 512
    expected_total = None
    seen = 0
    for i in range(nblocks):
        blk = data[i * 512:(i + 1) * 512]
        m0, m1, flags, addr, psize, bno, total, fam = struct.unpack_from("<8I", blk, 0)
        (mend,) = struct.unpack_from("<I", blk, 508)
        if m0 != UF2_MAGIC0 or m1 != UF2_MAGIC1 or mend != UF2_MAGIC_END:
            raise FirmwareError(f"UF2 block {i}: bad magic")
        if flags & UF2_FLAG_NOT_MAIN_FLASH:
            continue
        if not flags & UF2_FLAG_FAMILY or fam not in families:
            raise FirmwareError(f"UF2 block {i}: not an RP2040 image (family 0x{fam:08x})")
        if psize > 476 or psize == 0:
            raise FirmwareError(f"UF2 block {i}: bad payload size {psize}")
        if not (RP2040_FLASH_START <= addr and addr + psize <= RP2040_FLASH_END):
            raise FirmwareError(f"UF2 block {i}: address 0x{addr:08x} outside flash")
        if expected_total is None:
            expected_total = total
        if total != expected_total or bno != seen:
            raise FirmwareError(f"UF2 block {i}: inconsistent numbering")
        seen += 1
    if expected_total is None or seen != expected_total:
        raise FirmwareError("UF2 is truncated or contains no flash blocks")
    return nblocks


def make_uf2(payload: bytes, base: int = RP2040_FLASH_START, family: int = RP2040_FAMILY) -> bytes:
    """Build a UF2 from raw flash bytes (used by tests and for sanity tooling)."""
    chunks = [payload[i:i + 256] for i in range(0, len(payload), 256)] or [b""]
    out = bytearray()
    for i, c in enumerate(chunks):
        hdr = struct.pack("<8I", UF2_MAGIC0, UF2_MAGIC1, UF2_FLAG_FAMILY, base + 256 * i,
                          256, i, len(chunks), family)
        body = c.ljust(476, b"\x00")
        out += hdr + body + struct.pack("<I", UF2_MAGIC_END)
    return bytes(out)


def find_rp2_drive(globs: list[str]) -> Path | None:
    """A mounted RPI-RP2 drive is recognised by INFO_UF2.TXT mentioning RP2."""
    for pattern in globs:
        for cand in sorted(glob.glob(pattern)):
            info = Path(cand) / "INFO_UF2.TXT"
            try:
                text = info.read_text(errors="replace")
            except OSError:
                continue
            if "RP2" in text:
                return Path(cand)
    return None


Runner = Callable[..., subprocess.CompletedProcess]


class FirmwareFlasher:
    def __init__(self, link_factory: Callable[[], McuLink], drive_globs: list[str],
                 picotool: str = "auto", bootloader_timeout: float = 30.0,
                 reenumerate_timeout: float = 30.0, attempts: int = 3,
                 runner: Runner = subprocess.run, sleep: Callable[[float], None] = time.sleep,
                 poll_interval: float = 0.5):
        self.link_factory = link_factory
        self.drive_globs = drive_globs
        self.bootloader_timeout = bootloader_timeout
        self.reenumerate_timeout = reenumerate_timeout
        self.attempts = max(1, attempts)
        self.runner = runner
        self.sleep = sleep
        self.poll = poll_interval
        if picotool == "never":
            self.picotool = None
        elif picotool == "auto":
            self.picotool = shutil.which("picotool")
        else:
            self.picotool = picotool

    # -------------------------------------------------------------- helpers
    def _open_link(self) -> McuLink:
        link = self.link_factory()
        link.open()
        return link

    def current_version(self) -> str | None:
        try:
            link = self._open_link()
        except Exception as e:  # noqa: BLE001 - driver errors are opaque to us
            log.info("MCU not reachable: %s", e)
            return None
        try:
            return link.info().fw_version
        finally:
            _safe_close(link)

    def _in_bootloader(self) -> Path | str | None:
        drive = find_rp2_drive(self.drive_globs)
        if drive:
            return drive
        if self.picotool:
            r = self.runner([self.picotool, "info"], capture_output=True, text=True, timeout=10)
            if r.returncode == 0:
                return "picotool"
        return None

    def _wait(self, cond: Callable[[], Any], timeout: float, what: str) -> Any:
        deadline = time.monotonic() + timeout
        while True:
            v = cond()
            if v:
                return v
            if time.monotonic() >= deadline:
                raise FirmwareError(f"timed out after {timeout:.0f}s waiting for {what}")
            self.sleep(self.poll)

    def _enter_bootloader(self) -> Path | str:
        target = self._in_bootloader()
        if target:
            log.info("MCU already in bootloader (%s)", target)
            return target
        try:
            link = self._open_link()
        except Exception as e:  # noqa: BLE001
            raise FirmwareError(f"cannot open MCU link to enter bootloader: {e}") from e
        try:
            link.reboot_to_bootloader()
        except Exception as e:  # noqa: BLE001 - the port may vanish mid-call
            log.info("reboot_to_bootloader raised %r (often expected)", e)
        finally:
            _safe_close(link)
        return self._wait(self._in_bootloader, self.bootloader_timeout,
                          "the RP2040 bootloader (RPI-RP2 drive or picotool)")

    def _copy(self, target: Path | str, uf2: Path) -> None:
        if target == "picotool":
            r = self.runner([str(self.picotool), "load", "-v", "-x", str(uf2)],
                            capture_output=True, text=True, timeout=120)
            if r.returncode != 0:
                raise FirmwareError(f"picotool load failed: {(r.stderr or r.stdout).strip()}")
            return
        dst = Path(target) / "NEW.UF2"
        data = uf2.read_bytes()
        written = 0
        try:
            fd = os.open(dst, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
            try:
                for off in range(0, len(data), 64 * 1024):
                    written += os.write(fd, data[off:off + 64 * 1024])
                os.fsync(fd)
            finally:
                os.close(fd)
        except OSError as e:
            # The ROM reboots as soon as the last block arrives, so the drive
            # can vanish before fsync/close returns. Only fail if data is short.
            if written < len(data):
                raise FirmwareError(f"copy to {target} failed after {written} bytes: {e}") from e
            log.info("drive vanished after full write (expected): %s", e)

    def _reenumerated_version(self) -> str | None:
        if find_rp2_drive(self.drive_globs):
            return None
        return self.current_version()

    # ----------------------------------------------------------------- flash
    def flash(self, uf2: Path, expected_version: str,
              progress: Callable[[float, str], None] | None = None) -> str:
        uf2 = Path(uf2)
        validate_uf2(uf2.read_bytes())
        last_err: Exception | None = None
        for attempt in range(1, self.attempts + 1):
            def report(frac: float, msg: str) -> None:
                if progress:
                    progress(frac, f"{msg} (attempt {attempt}/{self.attempts})")
            try:
                report(0.1, "rebooting MCU into bootloader")
                target = self._enter_bootloader()
                report(0.4, f"writing firmware via {target}")
                self._copy(target, uf2)
                report(0.7, "waiting for MCU to restart")
                version = self._wait(self._reenumerated_version, self.reenumerate_timeout,
                                     "the MCU to re-enumerate with new firmware")
                if version != expected_version:
                    raise FirmwareError(
                        f"MCU reports firmware {version!r} after flashing, expected {expected_version!r}")
                report(1.0, f"firmware {version} verified")
                return version
            except FirmwareError as e:
                last_err = e
                log.warning("firmware flash attempt %d failed: %s", attempt, e)
        raise FirmwareError(f"firmware update failed after {self.attempts} attempts: {last_err}")


def _safe_close(link: McuLink) -> None:
    try:
        link.close()
    except Exception:  # noqa: BLE001
        pass
