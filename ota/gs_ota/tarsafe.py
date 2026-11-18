"""Safe extraction of release tarballs (.tar.zst / .tar).

Policy (anything else aborts the whole extraction):
* only regular files, directories and *relative* symlinks;
* no absolute names, no ``..`` components, no NUL/backslash, no duplicates;
* a member is never written through a symlink (every parent component is
  checked with lstat);
* symlink targets must stay inside the destination, both lexically and after
  the whole tree is extracted (realpath check);
* hardlinks, devices, FIFOs are rejected; setuid/setgid/sticky bits dropped,
  ownership not restored;
* caps on member count, path length and total unpacked bytes (zip-bomb guard).
The caller extracts into a fresh directory and deletes it on any error.
"""
from __future__ import annotations

import os
import posixpath
import stat
import tarfile
from pathlib import Path
from typing import BinaryIO, Callable

import zstandard

DEFAULT_MAX_BYTES = 4 * 1024 ** 3
DEFAULT_MAX_MEMBERS = 200_000
MAX_NAME = 4096


class UnsafeArchiveError(Exception):
    pass


def _clean_name(name: str) -> str:
    if not name or "\x00" in name or "\\" in name or len(name) > MAX_NAME:
        raise UnsafeArchiveError(f"invalid member name {name!r}")
    if name.startswith("/"):
        raise UnsafeArchiveError(f"absolute path in archive: {name!r}")
    parts = [p for p in name.split("/") if p not in ("", ".")]
    if any(p == ".." for p in parts):
        raise UnsafeArchiveError(f"path traversal in archive: {name!r}")
    return "/".join(parts)


def _open_stream(path: Path) -> tuple[BinaryIO, Callable[[], None]]:
    raw = open(path, "rb")
    magic = raw.read(4)
    raw.seek(0)
    if magic == b"\x28\xb5\x2f\xfd":  # zstd frame magic
        dctx = zstandard.ZstdDecompressor(max_window_size=2 ** 27)
        reader = dctx.stream_reader(raw)

        def close() -> None:
            reader.close()
            raw.close()
        return reader, close
    return raw, raw.close


def safe_extract(archive: Path, dest: Path, max_bytes: int = DEFAULT_MAX_BYTES,
                 max_members: int = DEFAULT_MAX_MEMBERS) -> list[str]:
    """Extract ``archive`` into the empty/non-existent directory ``dest``.
    Returns the list of member names extracted."""
    archive, dest = Path(archive), Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    if any(dest.iterdir()):
        raise UnsafeArchiveError(f"destination {dest} is not empty")
    root = os.path.realpath(dest)
    seen: set[str] = set()
    links: list[str] = []
    total = 0
    stream, close = _open_stream(archive)
    try:
        with tarfile.open(fileobj=stream, mode="r|") as tf:
            for count, m in enumerate(tf, 1):
                if count > max_members:
                    raise UnsafeArchiveError("too many archive members")
                name = _clean_name(m.name)
                if name == "":
                    if m.isdir():
                        continue
                    raise UnsafeArchiveError("member with empty name")
                if name in seen:
                    raise UnsafeArchiveError(f"duplicate member {name!r}")
                seen.add(name)
                target = os.path.join(root, name)
                _check_parents(root, name)
                if m.isdir():
                    os.makedirs(target, mode=0o755, exist_ok=True)
                    if os.path.islink(target) or not os.path.isdir(target):
                        raise UnsafeArchiveError(f"directory {name!r} collides")
                elif m.isreg():
                    total += m.size
                    if total > max_bytes:
                        raise UnsafeArchiveError("archive expands beyond size limit")
                    os.makedirs(os.path.dirname(target), mode=0o755, exist_ok=True)
                    src = tf.extractfile(m)
                    assert src is not None
                    mode = 0o755 if m.mode & 0o111 else 0o644
                    fd = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, mode)
                    written = 0
                    with os.fdopen(fd, "wb") as out:
                        while True:
                            buf = src.read(1024 * 1024)
                            if not buf:
                                break
                            written += len(buf)
                            out.write(buf)
                    if written != m.size:
                        raise UnsafeArchiveError(f"truncated member {name!r}")
                elif m.issym():
                    lt = m.linkname
                    if not lt or lt.startswith("/") or "\x00" in lt or "\\" in lt:
                        raise UnsafeArchiveError(f"unsafe symlink {name!r} -> {lt!r}")
                    resolved = posixpath.normpath(posixpath.join(posixpath.dirname(name), lt))
                    if resolved == ".." or resolved.startswith("../"):
                        raise UnsafeArchiveError(f"symlink escapes archive: {name!r} -> {lt!r}")
                    os.makedirs(os.path.dirname(target), mode=0o755, exist_ok=True)
                    os.symlink(lt, target)
                    links.append(name)
                else:
                    raise UnsafeArchiveError(
                        f"unsupported member type for {name!r} (hardlink/device/fifo)")
    except (tarfile.TarError, zstandard.ZstdError, EOFError) as e:
        raise UnsafeArchiveError(f"corrupt archive: {e}") from e
    except OSError as e:  # e.g. a member colliding with an existing path
        raise UnsafeArchiveError(f"cannot extract: {e}") from e
    finally:
        close()

    # Final check: the fully extracted tree is fixed now, so realpath of each
    # link is deterministic. Catches chains of individually-harmless links.
    for name in links:
        real = os.path.realpath(os.path.join(root, name))
        if real != root and not real.startswith(root + os.sep):
            raise UnsafeArchiveError(f"symlink resolves outside archive: {name!r}")
    return sorted(seen)


def _check_parents(root: str, name: str) -> None:
    cur = root
    for comp in name.split("/")[:-1]:
        cur = os.path.join(cur, comp)
        try:
            st = os.lstat(cur)
        except FileNotFoundError:
            return
        if stat.S_ISLNK(st.st_mode):
            raise UnsafeArchiveError(f"member {name!r} would be written through a symlink")
        if not stat.S_ISDIR(st.st_mode):
            raise UnsafeArchiveError(f"member {name!r}: parent is not a directory")
