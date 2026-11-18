"""HTTP(S)/file downloads: bounded fetches and resumable, verified artifact downloads."""
from __future__ import annotations

import hashlib
import os
import shutil
from pathlib import Path
from typing import Callable
from urllib.parse import unquote, urlparse

import httpx

from .fsutil import fsync_dir

CHUNK = 64 * 1024
DISK_MARGIN = 32 * 1024 * 1024

Progress = Callable[[int, int], None]  # (done_bytes, total_bytes)


class DownloadError(Exception):
    pass


class IntegrityError(DownloadError):
    """Downloaded bytes do not match the signed manifest (size or sha256)."""


class DiskSpaceError(DownloadError):
    pass


def make_client(timeout: float = 30.0, trust_env: bool = True) -> httpx.Client:
    return httpx.Client(timeout=httpx.Timeout(timeout, connect=10.0),
                        follow_redirects=True, trust_env=trust_env,
                        headers={"User-Agent": "gs-ota/1"})


def _file_path(url: str) -> Path:
    return Path(unquote(urlparse(url).path))


def fetch_bytes(client: httpx.Client, url: str, max_bytes: int) -> bytes:
    """GET a small resource (manifest/signature) with a hard size cap."""
    if url.startswith("file:"):
        p = _file_path(url)
        try:
            if p.stat().st_size > max_bytes:
                raise DownloadError(f"{url}: larger than {max_bytes} bytes")
            return p.read_bytes()
        except OSError as e:
            raise DownloadError(f"{url}: {e}") from e
    try:
        with client.stream("GET", url) as r:
            if r.status_code != 200:
                raise DownloadError(f"GET {url}: HTTP {r.status_code}")
            buf = bytearray()
            for chunk in r.iter_bytes():
                buf += chunk
                if len(buf) > max_bytes:
                    raise DownloadError(f"{url}: larger than {max_bytes} bytes")
            return bytes(buf)
    except httpx.HTTPError as e:
        raise DownloadError(f"GET {url}: {e}") from e


def check_free_space(directory: Path, needed: int, margin: int = DISK_MARGIN) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    free = shutil.disk_usage(directory).free
    if free < needed + margin:
        raise DiskSpaceError(
            f"not enough disk space in {directory}: need {needed + margin} bytes, have {free}")


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def verify_file(path: Path, sha256: str, size: int) -> None:
    st = path.stat()
    if st.st_size != size:
        raise IntegrityError(f"{path.name}: size {st.st_size} != expected {size}")
    got = sha256_file(path)
    if got != sha256:
        raise IntegrityError(f"{path.name}: sha256 mismatch (got {got})")


def download_artifact(client: httpx.Client, url: str, dest: Path, sha256: str, size: int,
                      progress: Progress | None = None,
                      should_abort: Callable[[], bool] | None = None,
                      margin: int = DISK_MARGIN) -> Path:
    """Download ``url`` to ``dest`` and verify size + sha256.

    The partial file ``dest + '.part'`` survives interruptions; the next call
    resumes it with an HTTP Range request. A server that ignores Range (HTTP
    200) restarts the download from zero. More bytes than ``size`` is treated
    as tampering. On any integrity failure the partial file is deleted.
    """
    dest = Path(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists():
        try:
            verify_file(dest, sha256, size)
            if progress:
                progress(size, size)
            return dest
        except IntegrityError:
            dest.unlink()
    part = dest.with_name(dest.name + ".part")
    have = part.stat().st_size if part.exists() else 0
    if have > size:
        part.unlink()
        have = 0
    check_free_space(dest.parent, size - have, margin)

    h = hashlib.sha256()
    if have:
        with open(part, "rb") as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b""):
                h.update(chunk)

    def write_stream(chunks, mode: str, start: int) -> int:
        done = start
        with open(part, mode) as f:
            for chunk in chunks:
                if should_abort and should_abort():
                    raise DownloadError("download aborted")
                if done + len(chunk) > size:
                    raise IntegrityError(f"{url}: server sent more than {size} bytes")
                f.write(chunk)
                h.update(chunk)
                done += len(chunk)
                if progress:
                    progress(done, size)
            f.flush()
            os.fsync(f.fileno())
        return done

    try:
        if have < size:
            if url.startswith("file:"):
                src = _file_path(url)
                with open(src, "rb") as s:
                    s.seek(have)
                    done = write_stream(iter(lambda: s.read(CHUNK), b""), "ab", have)
            else:
                headers = {"Range": f"bytes={have}-"} if have else {}
                with client.stream("GET", url, headers=headers) as r:
                    if have and r.status_code == 206:
                        cr = r.headers.get("content-range", "")
                        if not cr.startswith(f"bytes {have}-"):
                            raise DownloadError(f"{url}: unexpected Content-Range {cr!r}")
                        done = write_stream(r.iter_bytes(), "ab", have)
                    elif r.status_code == 200:
                        # Range not honoured (or fresh download): start over.
                        h = hashlib.sha256()
                        done = write_stream(r.iter_bytes(), "wb", 0)
                    elif r.status_code == 416:
                        part.unlink(missing_ok=True)
                        raise DownloadError(f"{url}: range not satisfiable; restarting next time")
                    else:
                        raise DownloadError(f"GET {url}: HTTP {r.status_code}")
        else:
            done = have
    except httpx.HTTPError as e:
        raise DownloadError(f"GET {url}: {e}") from e
    except IntegrityError:
        part.unlink(missing_ok=True)
        raise

    if done != size:
        # Truncated transfer: keep the partial file for a later resume.
        raise DownloadError(f"{url}: incomplete download ({done}/{size} bytes)")
    if h.hexdigest() != sha256:
        part.unlink(missing_ok=True)
        raise IntegrityError(f"{url}: sha256 mismatch")
    os.replace(part, dest)
    fsync_dir(dest.parent)
    return dest
