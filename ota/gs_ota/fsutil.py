"""Crash-safe filesystem helpers (atomic writes and symlink switches)."""
from __future__ import annotations

import json
import os
import shutil
from pathlib import Path
from typing import Any


def fsync_dir(path: Path) -> None:
    try:
        fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    except OSError:
        return
    try:
        os.fsync(fd)
    except OSError:
        pass
    finally:
        os.close(fd)


def atomic_write_bytes(path: Path, data: bytes, mode: int = 0o644) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp{os.getpid()}")
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, mode)
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    fsync_dir(path.parent)


def atomic_write_json(path: Path, obj: Any) -> None:
    atomic_write_bytes(path, (json.dumps(obj, indent=2, sort_keys=True) + "\n").encode())


def atomic_symlink(target: str, link: Path) -> None:
    """Point ``link`` at ``target`` atomically: create a temp symlink and
    rename(2) it over the old one, so readers always see old or new."""
    link = Path(link)
    tmp = link.with_name(f".{link.name}.new{os.getpid()}")
    try:
        os.unlink(tmp)
    except FileNotFoundError:
        pass
    os.symlink(target, tmp)
    os.replace(tmp, link)
    fsync_dir(link.parent)


def rmtree(path: Path) -> None:
    path = Path(path)
    if path.is_symlink() or path.is_file():
        path.unlink()
    elif path.exists():
        # make read-only dirs deletable
        for root, dirs, _files in os.walk(path):
            for d in dirs:
                p = os.path.join(root, d)
                if not os.path.islink(p):
                    try:
                        os.chmod(p, 0o755)
                    except OSError:
                        pass
        shutil.rmtree(path)
