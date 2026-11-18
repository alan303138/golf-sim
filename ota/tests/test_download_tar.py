import io
import os
import tarfile
from pathlib import Path

import pytest
import zstandard

from gs_ota.devserver import RangeHandler
from gs_ota.download import (DiskSpaceError, DownloadError, IntegrityError, download_artifact,
                             make_client, sha256_file)
from gs_ota.tarsafe import UnsafeArchiveError, safe_extract


# ----------------------------------------------------------------- downloads
@pytest.fixture
def blob(server):
    srv, base = server
    data = os.urandom(300_000)
    (srv / "a.bin").write_bytes(data)
    import hashlib
    return data, hashlib.sha256(data).hexdigest(), base + "/a.bin"


def test_download_ok(blob, tmp_path):
    data, sha, url = blob
    seen = []
    out = download_artifact(make_client(trust_env=False), url, tmp_path / "a.bin", sha, len(data),
                            progress=lambda d, t: seen.append(d))
    assert out.read_bytes() == data and seen[-1] == len(data)


def test_truncated_download_resumes_with_range(blob, tmp_path):
    data, sha, url = blob
    ranges = []

    def cut(handler, src, out):
        ranges.append(handler.headers.get("Range"))
        if len(ranges) == 1:
            out.write(src.read(100_000))      # then the connection drops
            handler.close_connection = True
            return
        RangeHandler.fault_hook = None
        left = getattr(handler, "_range_left", None)
        out.write(src.read(left) if left else src.read())
    RangeHandler.fault_hook = cut
    c = make_client(trust_env=False)
    with pytest.raises(DownloadError):
        download_artifact(c, url, tmp_path / "a.bin", sha, len(data))
    part = tmp_path / "a.bin.part"
    have = part.stat().st_size
    assert 0 < have <= 100_000
    RangeHandler.fault_hook = cut
    out = download_artifact(c, url, tmp_path / "a.bin", sha, len(data))
    assert ranges == [None, f"bytes={have}-"]
    assert out.read_bytes() == data and not part.exists()


def test_tampered_artifact_rejected_and_part_removed(blob, server, tmp_path):
    data, sha, url = blob
    srv, _ = server
    t = bytearray(data)
    t[1234] ^= 0xFF
    (srv / "a.bin").write_bytes(bytes(t))
    with pytest.raises(IntegrityError):
        download_artifact(make_client(trust_env=False), url, tmp_path / "a.bin", sha, len(data))
    assert not (tmp_path / "a.bin").exists() and not (tmp_path / "a.bin.part").exists()


def test_oversize_and_truncated_server_file(blob, server, tmp_path):
    data, sha, url = blob
    srv, _ = server
    c = make_client(trust_env=False)
    (srv / "a.bin").write_bytes(data + b"extra")
    with pytest.raises(IntegrityError):
        download_artifact(c, url, tmp_path / "a.bin", sha, len(data))
    (srv / "a.bin").write_bytes(data[:-10])           # server file truncated
    with pytest.raises(DownloadError):
        download_artifact(c, url, tmp_path / "b.bin", sha, len(data))
    assert not (tmp_path / "b.bin").exists()


def test_existing_corrupt_part_detected(blob, tmp_path):
    data, sha, url = blob
    (tmp_path / "a.bin.part").write_bytes(b"\x00" * 1000)   # garbage prefix
    with pytest.raises(IntegrityError):
        download_artifact(make_client(trust_env=False), url, tmp_path / "a.bin", sha, len(data))
    out = download_artifact(make_client(trust_env=False), url, tmp_path / "a.bin", sha, len(data))
    assert sha256_file(out) == sha


def test_disk_space_check(blob, tmp_path, monkeypatch):
    data, sha, url = blob
    import shutil
    from collections import namedtuple
    U = namedtuple("U", "total used free")
    monkeypatch.setattr(shutil, "disk_usage", lambda p: U(10, 10, 1000))
    with pytest.raises(DiskSpaceError):
        download_artifact(make_client(trust_env=False), url, tmp_path / "a.bin", sha, len(data))


def test_file_url_download(tmp_path):
    src = tmp_path / "usb" / "x.bin"
    src.parent.mkdir()
    src.write_bytes(b"hello" * 1000)
    out = download_artifact(make_client(), src.as_uri(), tmp_path / "dl" / "x.bin",
                            sha256_file(src), src.stat().st_size)
    assert out.read_bytes() == src.read_bytes()


# ------------------------------------------------------------------- tarsafe
def _tar(members, zst=True) -> bytes:
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w") as tf:
        for m in members:
            kind, name = m[0], m[1]
            ti = tarfile.TarInfo(name)
            if kind == "file":
                payload = m[2] if len(m) > 2 else b"x"
                ti.size = len(payload)
                tf.addfile(ti, io.BytesIO(payload))
            elif kind == "dir":
                ti.type = tarfile.DIRTYPE
                tf.addfile(ti)
            elif kind == "sym":
                ti.type, ti.linkname = tarfile.SYMTYPE, m[2]
                tf.addfile(ti)
            elif kind == "hard":
                ti.type, ti.linkname = tarfile.LNKTYPE, m[2]
                tf.addfile(ti)
            elif kind == "dev":
                ti.type = tarfile.CHRTYPE
                tf.addfile(ti)
            elif kind == "suid":
                ti.mode = 0o4755
                ti.size = 1
                tf.addfile(ti, io.BytesIO(b"x"))
    raw = buf.getvalue()
    return zstandard.ZstdCompressor().compress(raw) if zst else raw


def _extract(tmp_path, members, **kw):
    tmp_path.mkdir(parents=True, exist_ok=True)
    arc = tmp_path / "t.tar.zst"
    arc.write_bytes(_tar(members))
    dest = tmp_path / "out"
    return safe_extract(arc, dest, **kw), dest


@pytest.mark.parametrize("members", [
    [("file", "../evil")],
    [("file", "a/../../evil")],
    [("file", "/etc/evil")],
    [("sym", "l", "/etc")],
    [("sym", "l", "../..")],
    [("dir", "d"), ("sym", "d/l", "../../x")],
    # each link lexically fine, but the chain escapes: a/b/s -> root, x -> a/b/s/..
    [("dir", "a"), ("dir", "a/b"), ("sym", "a/b/s", "../.."), ("sym", "x", "a/b/s/..")],
    # write through a symlink that points inside, then a file below it
    [("dir", "real"), ("sym", "l", "real"), ("file", "l/f")],
    [("hard", "h", "etc/passwd")],
    [("dev", "null")],
    [("file", "a"), ("file", "a")],
    [("file", "a\\..\\b")],
])
def test_unsafe_archives_rejected(tmp_path, members):
    with pytest.raises(UnsafeArchiveError):
        _extract(tmp_path, members)
    assert not (tmp_path / "evil").exists()


def test_safe_archive_extracts(tmp_path):
    names, dest = _extract(tmp_path, [("dir", "bin"), ("file", "bin/run", b"#!/bin/sh\n"),
                                      ("dir", "lib"), ("sym", "lib/cur", "../bin/run"),
                                      ("suid", "bin/s")])
    assert (dest / "bin/run").read_bytes() == b"#!/bin/sh\n"
    assert os.readlink(dest / "lib/cur") == "../bin/run"
    assert not (dest / "bin/s").stat().st_mode & 0o7000   # setuid stripped
    assert "bin/s" in names


def test_size_and_member_limits(tmp_path):
    with pytest.raises(UnsafeArchiveError):
        _extract(tmp_path, [("file", "big", b"x" * 5000)], max_bytes=1000)
    with pytest.raises(UnsafeArchiveError):
        _extract(tmp_path / "2", [("file", f"f{i}") for i in range(20)], max_members=10)


def test_corrupt_archive(tmp_path):
    arc = tmp_path / "t.tar.zst"
    good = _tar([("file", "a", b"y" * 10000)])
    arc.write_bytes(good[: len(good) // 2])
    with pytest.raises(UnsafeArchiveError):
        safe_extract(arc, tmp_path / "o")
