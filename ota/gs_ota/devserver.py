"""Tiny static file server with HTTP Range support, for LAN testing.

    python -m gs_ota.devserver --dir releases --port 8000 [--bind 0.0.0.0]

Releases are plain static files, so production hosting can be any static
host (GitHub Pages/Releases, S3, nginx). This server exists because Python's
http.server does not support Range (needed to test resumed downloads).
Not hardened for the Internet; development use only.
"""
from __future__ import annotations

import argparse
import http.server
import os
import re
import threading
from functools import partial
from pathlib import Path

_RANGE_RE = re.compile(r"^bytes=(\d+)-(\d*)$")


class RangeHandler(http.server.SimpleHTTPRequestHandler):
    # Hooks used by tests to simulate faults: callable(handler, path) -> None
    fault_hook = None

    def log_message(self, fmt: str, *args) -> None:  # quieter
        if os.environ.get("GS_OTA_DEVSERVER_LOG"):
            super().log_message(fmt, *args)

    def send_head(self):  # type: ignore[override]
        path = self.translate_path(self.path)
        if os.path.isdir(path):
            return super().send_head()
        rng = self.headers.get("Range")
        if not rng or not os.path.isfile(path):
            return super().send_head()
        m = _RANGE_RE.match(rng.strip())
        size = os.path.getsize(path)
        if not m:
            return super().send_head()
        start = int(m.group(1))
        end = int(m.group(2)) if m.group(2) else size - 1
        if start >= size or end < start:
            self.send_response(416)
            self.send_header("Content-Range", f"bytes */{size}")
            self.end_headers()
            return None
        end = min(end, size - 1)
        f = open(path, "rb")
        f.seek(start)
        self.send_response(206)
        self.send_header("Content-Type", self.guess_type(path))
        self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.send_header("Content-Length", str(end - start + 1))
        self.send_header("Accept-Ranges", "bytes")
        self.end_headers()
        self._range_left = end - start + 1
        return f

    def copyfile(self, source, outputfile) -> None:  # type: ignore[override]
        if RangeHandler.fault_hook:
            RangeHandler.fault_hook(self, source, outputfile)
            return
        left = getattr(self, "_range_left", None)
        if left is None:
            return super().copyfile(source, outputfile)
        while left > 0:
            buf = source.read(min(65536, left))
            if not buf:
                break
            outputfile.write(buf)
            left -= len(buf)


def serve(directory: str | Path, host: str = "127.0.0.1", port: int = 0,
          handler: type[RangeHandler] = RangeHandler) -> tuple[http.server.ThreadingHTTPServer, threading.Thread]:
    """Start in a background thread; returns (server, thread). Port 0 = any."""
    srv = http.server.ThreadingHTTPServer((host, port), partial(handler, directory=str(directory)))
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    return srv, t


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="gs_ota.devserver")
    ap.add_argument("--dir", default=".")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)
    a = ap.parse_args(argv)
    srv = http.server.ThreadingHTTPServer((a.bind, a.port), partial(RangeHandler, directory=a.dir))
    print(f"serving {Path(a.dir).resolve()} on http://{a.bind}:{a.port}/ (Range supported)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
