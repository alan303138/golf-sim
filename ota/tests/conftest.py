from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

import pytest

OTA = Path(__file__).resolve().parents[1]
REPO = OTA.parent
for p in (REPO / "common", OTA):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from gs_common.interfaces import McuInfo, McuLink  # noqa: E402

from gs_ota import keys as K  # noqa: E402
from gs_ota.bundle import BundleConfig, DataSpec, PackageSpec, build_tree, write_tar_zst  # noqa: E402
from gs_ota.config import AgentConfig  # noqa: E402
from gs_ota.devserver import RangeHandler, serve  # noqa: E402
from gs_ota.download import sha256_file  # noqa: E402
from gs_ota.engine import UpdateEngine  # noqa: E402
from gs_ota.manifest import build_manifest_obj, dump_manifest  # noqa: E402
from gs_ota.mcu import make_uf2  # noqa: E402
from gs_ota.services import NullServiceController  # noqa: E402
from gs_ota.slots import SlotManager  # noqa: E402


class Crash(BaseException):
    """Simulated power loss / kill -9: escapes every `except Exception`."""


@pytest.fixture(scope="session")
def keypair():
    ed = K.generate_ed25519()
    rs = K.generate_rsa()
    return ed, rs


@pytest.fixture(scope="session")
def bundle_cache(tmp_path_factory):
    """Builds sensor bundles (copy mode, real gs_common + gs_ota) once per version."""
    root = tmp_path_factory.mktemp("bundles")
    web = root / "web"
    web.mkdir()
    (web / "index.html").write_text("<html>pwa</html>")
    built: dict[str, Path] = {}

    def get(version: str) -> Path:
        if version not in built:
            cfg = BundleConfig(base=OTA, python="%d.%d" % sys.version_info[:2], mode="copy",
                               packages=[PackageSpec(REPO / "common", "gs_common", True),
                                         PackageSpec(OTA, "gs_ota", True),
                                         PackageSpec(root / "missing", "gs_service")],
                               data=[DataSpec(web, "share/web")],
                               launchers={"gs-ota-agent": "gs_ota.agent"},
                               smoke_imports=["gs_common", "gs_ota.engine"])
            tree = root / f"tree-{version}"
            tree.mkdir()
            build_tree(cfg, version, tree)
            out = root / f"sensor-{version}.tar.zst"
            write_tar_zst(tree, out, level=3)
            built[version] = out
        return built[version]
    return get


def fw_bytes(version: str) -> bytes:
    return make_uf2(b"GSFW:" + version.encode() + b"\x00" + bytes(range(256)) * 3)


class Release:
    """Writes signed releases into <srv>/<channel>/."""

    def __init__(self, srv: Path, keypair, bundle_cache):
        self.srv, self.keypair, self.bundles = srv, keypair, bundle_cache

    def publish(self, sensor: str | None = "1.1.0", firmware: str | None = None,
                channel: str = "stable", released: str = "2026-11-16T00:00:00Z",
                sensor_min: str | None = None, sign_key=None, manifest_channel: str | None = None,
                sim: str | None = None) -> Path:
        d = self.srv / channel
        d.mkdir(parents=True, exist_ok=True)
        arts = {}
        if sensor:
            src = self.bundles(sensor)
            dst = d / src.name
            dst.write_bytes(src.read_bytes())
            arts["sensor"] = {"version": sensor, "url": dst.name, "sha256": sha256_file(dst),
                              "size": dst.stat().st_size}
            if sensor_min:
                arts["sensor"]["min_version"] = sensor_min
        if firmware:
            dst = d / f"timing-mcu-{firmware}.uf2"
            dst.write_bytes(fw_bytes(firmware))
            arts["firmware"] = {"version": firmware, "url": dst.name,
                                "sha256": sha256_file(dst), "size": dst.stat().st_size}
        if sim:
            dst = d / f"sim-{sim}.pck"
            dst.write_bytes(b"GDPC" + b"x" * 1000)
            arts["sim"] = {"version": sim, "url": dst.name, "sha256": sha256_file(dst),
                           "size": dst.stat().st_size, "platform": "windows-x86_64"}
        data = dump_manifest(build_manifest_obj(manifest_channel or channel, released, arts,
                                                "notes"))
        (d / "manifest.json").write_bytes(data)
        ed, rs = self.keypair
        (d / "manifest.json.sig").write_bytes(K.sign_ed25519(sign_key or ed, data))
        (d / "manifest.json.rsasig").write_bytes(K.sign_rsa(rs, data))
        return d


@pytest.fixture
def server(tmp_path):
    srv = tmp_path / "srv"
    srv.mkdir()
    httpd, _t = serve(srv)
    yield srv, f"http://127.0.0.1:{httpd.server_address[1]}"
    RangeHandler.fault_hook = None
    httpd.shutdown()
    httpd.server_close()


@pytest.fixture
def release(server, keypair, bundle_cache):
    return Release(server[0], keypair, bundle_cache)


class FakeService(NullServiceController):
    """Records calls; the 'service' runs whatever `current` points to."""

    def __init__(self) -> None:
        super().__init__()
        self.crash_on: str | None = None
        self.crash_count = 0

    def _maybe_crash(self, what: str) -> None:
        if self.crash_on == what and self.crash_count > 0:
            self.crash_count -= 1
            raise Crash(what)

    def start(self) -> None:
        super().start()
        self._maybe_crash("start")

    def stop(self) -> None:
        super().stop()
        self._maybe_crash("stop")

    def restart(self) -> None:
        super().restart()
        self._maybe_crash("restart")


class SlotHealth:
    """Health probe that reports the version in the `current` slot, like a
    real gs_service restarted from that slot would. `broken` versions never
    become healthy."""

    def __init__(self, root: Path):
        self.slots = SlotManager(root)
        self.broken: set[str] = set()
        self.crash_times = 0
        self.calls = 0

    def __call__(self, expected: str):
        self.calls += 1
        if self.crash_times > 0:
            self.crash_times -= 1
            raise Crash("health")
        v = self.slots.version(self.slots.current())
        if v in self.broken:
            return False, f"{v} crashed"
        return v == expected, f"reports {v}"


@pytest.fixture
def python_env(monkeypatch):
    monkeypatch.setenv("GS_PYTHON", sys.executable)


@pytest.fixture
def make_engine(tmp_path, server, keypair, python_env):
    srv, base = server
    root = tmp_path / "opt"
    state = tmp_path / "state"
    pub = tmp_path / "pub.pem"
    pub.write_bytes(K.public_pem(keypair[0]))

    def make(service=None, health=None, flasher=None, **cfg_kw):
        cfg = AgentConfig(manifest_url=base + "/{channel}/manifest.json", root=str(root),
                          state_dir=str(state), trusted_keys=[str(pub)], service_control="none",
                          health_timeout_s=1.0, health_interval_s=0.01, trust_env_proxy=False,
                          check_interval_s=0, update_firmware=False, **cfg_kw)
        svc = service or FakeService()
        hp = health or SlotHealth(root)
        eng = UpdateEngine(cfg, service=svc, health_probe=hp, flasher=flasher,
                           fw_version_source=lambda: None, sleep=lambda s: None)
        return eng, svc, hp
    make.root = root
    make.state = state
    return make


@pytest.fixture
def bootstrapped(make_engine, release, tmp_path):
    """Device with sensor 1.0.0 installed in slot a via bootstrap."""
    d = release.publish(sensor="1.0.0", channel="bootstrap", released="2026-01-01T00:00:00Z")
    eng, svc, hp = make_engine()
    eng.bootstrap(d / "manifest.json")
    return eng, svc, hp


# ----------------------------------------------------------- fake MCU + drive
class FakeBoard:
    """Simulates an RP2040: CDC app mode with a version, or BOOTSEL mode with
    an RPI-RP2 drive directory. Writing NEW.UF2 'flashes' it."""

    def __init__(self, drive_dir: Path, version: str = "1.0.0"):
        self.drive_dir = drive_dir
        self.version = version
        self.mode = "app"
        self.reboots = 0
        self.flashes = 0
        self.ignore_bootloader_cmd = False
        self.flash_result_override: str | None = None
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._watch, daemon=True)
        self._t.start()

    def enter_bootsel(self) -> None:
        self.mode = "boot"
        self.drive_dir.mkdir(parents=True, exist_ok=True)
        (self.drive_dir / "INFO_UF2.TXT").write_text(
            "UF2 Bootloader v3.0\nModel: Raspberry Pi RP2\nBoard-ID: RPI-RP2\n")

    def _watch(self) -> None:
        while not self._stop.is_set():
            f = self.drive_dir / "NEW.UF2"
            if self.mode == "boot" and f.exists():
                time.sleep(0.05)
                data = f.read_bytes()
                payload = b"".join(data[i + 32:i + 32 + 256] for i in range(0, len(data), 512))
                ver = payload[5:payload.index(b"\x00", 5)].decode()
                for p in self.drive_dir.iterdir():
                    p.unlink()
                self.drive_dir.rmdir()
                self.flashes += 1
                self.version = self.flash_result_override or ver
                self.mode = "app"
            time.sleep(0.01)

    def close(self) -> None:
        self._stop.set()


class FakeLink(McuLink):
    def __init__(self, board: FakeBoard):
        self.board = board
        self.opened = False

    def open(self):
        if self.board.mode != "app":
            raise OSError("no such device /dev/ttyACM0")
        self.opened = True

    def close(self):
        self.opened = False

    def info(self):
        assert self.opened
        return McuInfo(fw_version=self.board.version, hw_revision=1, serial="E660", uptime_ms=5)

    def reboot_to_bootloader(self):
        assert self.opened
        self.board.reboots += 1
        if not self.board.ignore_bootloader_cmd:
            self.board.enter_bootsel()

    def configure(self, config): ...
    def start_triggers(self): ...
    def stop_triggers(self): ...
    def arm_radar(self, armed): ...
    def force_shot(self): ...
    def on_shot(self, callback): ...
    def frame_counter(self): return 0


@pytest.fixture
def board(tmp_path):
    b = FakeBoard(tmp_path / "media" / "RPI-RP2")
    yield b
    b.close()


def read_json(p: Path):
    return json.loads(Path(p).read_text())
