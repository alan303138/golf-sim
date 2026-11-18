"""OTA agent daemon and admin CLI.

    python -m gs_ota.agent [--config /etc/golfsim/ota.toml] run
    python -m gs_ota.agent status|check|apply
    python -m gs_ota.agent bootstrap --manifest DIR/manifest.json [--bundle FILE]
    python -m gs_ota.agent flash-firmware --manifest DIR/manifest.json [--uf2 FILE]

``run`` recovers any interrupted transaction first (boot-count rollback),
then serves the control socket used by gs_service and runs the optional
periodic check. Control protocol: newline-delimited JSON over a Unix stream
socket; requests ``{"op": "state"|"check"|"apply"|"subscribe"}``, responses
``{"ok": true, "state": {...}}`` / ``{"ok": false, "error": "..."}``;
``subscribe`` then streams ``{"event": "state", "state": {...}}`` lines.
Access control is the socket's file mode/group (see deploy/).
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import os
import queue
import signal
import socketserver
import sys
import threading
from pathlib import Path
from typing import Any

from gs_common.interfaces import UpdateState

from .config import DEFAULT_CONFIG_PATH, AgentConfig
from .engine import UpdateEngine, UpdateError
from .updater import OtaUpdater

log = logging.getLogger("gs_ota.agent")


def state_to_dict(s: UpdateState) -> dict[str, Any]:
    return dataclasses.asdict(s)


class _Fanout:
    def __init__(self) -> None:
        self.queues: set[queue.Queue] = set()
        self.lock = threading.Lock()

    def __call__(self, st: UpdateState) -> None:
        with self.lock:
            for q in self.queues:
                try:
                    q.put_nowait(st)
                except queue.Full:
                    pass


class ControlServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, path: str, updater: OtaUpdater, mode: int = 0o660):
        self.updater = updater
        self.fanout = _Fanout()
        updater.on_change(self.fanout)
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        if p.exists() or p.is_symlink():
            p.unlink()
        super().__init__(str(p), _Handler)
        os.chmod(p, mode)


class _Handler(socketserver.StreamRequestHandler):
    server: ControlServer

    def _send(self, obj: dict) -> None:
        self.wfile.write((json.dumps(obj) + "\n").encode())
        self.wfile.flush()

    def handle(self) -> None:
        upd = self.server.updater
        for raw in self.rfile:
            if len(raw) > 4096:
                self._send({"ok": False, "error": "request too long"})
                return
            try:
                req = json.loads(raw)
                op = req.get("op") if isinstance(req, dict) else None
            except ValueError:
                self._send({"ok": False, "error": "bad json"})
                continue
            try:
                if op == "state":
                    self._send({"ok": True, "state": state_to_dict(upd.state())})
                elif op == "check":
                    st = upd.state() if upd.busy else upd.engine.check()
                    self._send({"ok": True, "state": state_to_dict(st)})
                elif op == "apply":
                    upd.start_apply()
                    self._send({"ok": True, "state": state_to_dict(upd.state())})
                elif op == "subscribe":
                    self._subscribe()
                    return
                else:
                    self._send({"ok": False, "error": f"unknown op {op!r}"})
            except UpdateError as e:
                self._send({"ok": False, "error": str(e)})

    def _subscribe(self) -> None:
        q: queue.Queue = queue.Queue(maxsize=256)
        fan = self.server.fanout
        with fan.lock:
            fan.queues.add(q)
        try:
            self._send({"ok": True, "state": state_to_dict(self.server.updater.state())})
            while True:
                try:
                    st = q.get(timeout=15)
                    self._send({"event": "state", "state": state_to_dict(st)})
                except queue.Empty:
                    self._send({"event": "keepalive"})
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with fan.lock:
                fan.queues.discard(q)


def build_engine(cfg: AgentConfig, **kw: Any) -> UpdateEngine:
    return UpdateEngine(cfg, **kw)


def run_daemon(cfg: AgentConfig, engine: UpdateEngine | None = None,
               stop: threading.Event | None = None) -> int:
    restart = threading.Event()
    stop = stop or threading.Event()
    if engine is None:
        engine = build_engine(cfg, on_committed=lambda v: restart.set())
    result = engine.recover()
    log.info("startup recovery: %s", result)
    updater = OtaUpdater(engine)
    server = ControlServer(cfg.socket, updater)
    t = threading.Thread(target=server.serve_forever, name="gs-ota-ctl", daemon=True)
    t.start()
    updater.start_periodic(cfg.check_interval_s, cfg.auto_apply)
    log.info("gs-ota agent listening on %s", cfg.socket)
    try:
        while not stop.is_set():
            if restart.is_set() and not updater.busy:
                # New committed release: exit so systemd restarts us from it.
                log.info("committed a new release; restarting agent")
                break
            stop.wait(1.0)
    finally:
        updater.close()
        server.shutdown()
        server.server_close()
    return 0


def _client_call(cfg: AgentConfig, op: str) -> dict:
    import socket
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(300)
    s.connect(cfg.socket)
    s.sendall((json.dumps({"op": op}) + "\n").encode())
    line = s.makefile("rb").readline()
    s.close()
    return json.loads(line)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="gs-ota-agent")
    ap.add_argument("--config", default=os.environ.get("GS_OTA_CONFIG", str(DEFAULT_CONFIG_PATH)))
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("run", help="run the daemon")
    for op in ("status", "check", "apply"):
        sub.add_parser(op, help=f"{op} via the running daemon's socket")
    b = sub.add_parser("bootstrap", help="first install from a local signed release")
    b.add_argument("--manifest", required=True)
    b.add_argument("--bundle")
    f = sub.add_parser("flash-firmware", help="(re)flash firmware from a local signed release")
    f.add_argument("--manifest", required=True)
    f.add_argument("--uf2")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = AgentConfig.load(args.config)

    if args.cmd == "run":
        stop = threading.Event()
        signal.signal(signal.SIGTERM, lambda *_: stop.set())
        signal.signal(signal.SIGINT, lambda *_: stop.set())
        return run_daemon(cfg, stop=stop)
    if args.cmd in ("status", "check", "apply"):
        print(json.dumps(_client_call(cfg, "state" if args.cmd == "status" else args.cmd), indent=2))
        return 0
    engine = build_engine(cfg)
    if args.cmd == "bootstrap":
        print("installed", engine.bootstrap(Path(args.manifest), Path(args.bundle) if args.bundle else None))
        return 0
    if args.cmd == "flash-firmware":
        return _flash_local(engine, Path(args.manifest), args.uf2)
    return 2


def _flash_local(engine: UpdateEngine, manifest: Path, uf2: str | None) -> int:
    from urllib.parse import urlparse

    from .download import verify_file
    from .manifest import verify_manifest
    m = verify_manifest(manifest.read_bytes(), manifest.with_name(manifest.name + ".sig").read_bytes(),
                        engine.trusted_keys())
    a = m.artifacts.get("firmware")
    if a is None or engine.flasher is None:
        print("no firmware artifact or no MCU link", file=sys.stderr)
        return 1
    path = Path(uf2) if uf2 else manifest.parent / Path(urlparse(a.url).path).name
    verify_file(path, a.sha256, a.size)
    engine.service.stop()
    try:
        engine._flash(path, a.version)
    finally:
        engine.service.start()
    print("firmware", a.version, "verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
