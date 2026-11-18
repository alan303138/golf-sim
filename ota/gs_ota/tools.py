"""Release tooling.

    python -m gs_ota.tools keygen        --out ~/golfsim-keys
    python -m gs_ota.tools build-sensor  --version 1.2.0 --out releases/stable
    python -m gs_ota.tools manifest      --dir releases/stable --channel stable \\
            --sensor releases/stable/sensor-1.2.0.tar.zst --sensor-min 1.0.0 \\
            --firmware build/timing-mcu.uf2 --firmware-version 1.1.0 \\
            --sim build/sim-1.3.0.pck --sim-platform windows-x86_64 --notes-file NOTES.md
    python -m gs_ota.tools sign          --dir releases/stable --key-dir ~/golfsim-keys
    python -m gs_ota.tools verify        --dir releases/stable --pub-dir ota/keys
    python -m gs_ota.tools serve         --dir releases --port 8000

Private keys never belong in the repository: keygen refuses to write into a
git work tree unless --i-know-what-im-doing is given.
"""
from __future__ import annotations

import argparse
import getpass
import json
import os
import re
import shutil
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

from . import keys as K
from .download import sha256_file, verify_file
from .manifest import (MANIFEST_NAME, RSASIG_NAME, SIG_NAME, build_manifest_obj, dump_manifest,
                       parse_manifest_bytes, verify_manifest)
from .mcu import validate_uf2
from .semver import Version
from .tarsafe import safe_extract

HERE = Path(__file__).resolve().parent
DEFAULT_BUNDLE_CONFIG = HERE.parent / "release" / "sensor-bundle.toml"
_VER_IN_NAME = re.compile(r"-(\d+\.\d+\.\d+(?:-[0-9A-Za-z.-]+)?)(?:\.tar\.zst|\.[A-Za-z0-9]+)$")


def _die(msg: str) -> "None":
    print(f"error: {msg}", file=sys.stderr)
    raise SystemExit(1)


def _inside_git(path: Path) -> bool:
    p = path.resolve()
    for d in (p, *p.parents):
        if (d / ".git").exists():
            return True
    return False


def _passphrase(env: str | None, confirm: bool) -> bytes | None:
    if env:
        v = os.environ.get(env)
        if v is None:
            _die(f"environment variable {env} not set")
        return v.encode() if v else None
    if not sys.stdin.isatty():
        return None
    p = getpass.getpass("key passphrase (empty = unencrypted): ")
    if confirm and p and getpass.getpass("repeat: ") != p:
        _die("passphrases differ")
    return p.encode() if p else None


# ------------------------------------------------------------------ keygen
def cmd_keygen(a: argparse.Namespace) -> None:
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if _inside_git(out) and not a.i_know_what_im_doing:
        _die(f"{out} is inside a git work tree; private keys must stay out of the repo")
    os.chmod(out, 0o700)
    pw = _passphrase(a.passphrase_env, confirm=True)
    ed = K.generate_ed25519()
    rs = K.generate_rsa()
    K.write_secret(out / K.ED25519_PRIV_NAME, K.private_pem(ed, pw))
    K.write_secret(out / K.RSA_PRIV_NAME, K.private_pem(rs, pw))
    (out / K.ED25519_PUB_NAME).write_bytes(K.public_pem(ed))
    (out / K.RSA_PUB_NAME).write_bytes(K.public_pem(rs))
    print(f"wrote keys to {out}")
    print(f"  Ed25519 key id {K.key_id(ed.public_key())}  RSA key id {K.key_id(rs.public_key())}")
    print("copy ONLY the *.pub.pem files into ota/keys/ (device image) and sim/ (simulator)")


# ------------------------------------------------------------ build-sensor
def cmd_build_sensor(a: argparse.Namespace) -> None:
    from .bundle import build_sensor_bundle
    out = build_sensor_bundle(a.config, a.version, a.out, strict=a.strict,
                              wheelhouse=a.wheelhouse, mode=a.mode)
    print(f"built {out} ({out.stat().st_size} bytes, sha256 {sha256_file(out)})")


# ---------------------------------------------------------------- manifest
def _version_from_name(path: Path) -> str | None:
    m = _VER_IN_NAME.search(path.name)
    return m.group(1) if m else None


def _sensor_version(path: Path) -> str | None:
    with tempfile.TemporaryDirectory() as td:
        safe_extract(path, Path(td) / "x")
        try:
            return json.loads((Path(td) / "x" / "release.json").read_text())["version"]
        except (OSError, ValueError, KeyError):
            return None


def _add(arts: dict, d: Path, name: str, file: str, version: str | None,
         extra: dict) -> None:
    src = Path(file)
    if not src.is_file():
        _die(f"{src} not found")
    version = version or _version_from_name(src)
    if name == "sensor":
        bundled = _sensor_version(src)
        if version and bundled and version != bundled:
            _die(f"sensor version {version} != release.json {bundled}")
        version = version or bundled
    if not version:
        _die(f"cannot infer {name} version from {src.name}; pass --{name}-version")
    Version.parse(version)
    if name == "firmware":
        validate_uf2(src.read_bytes())
    dst = d / src.name
    if src.resolve() != dst.resolve():
        shutil.copy2(src, dst)
    arts[name] = {"version": version, "url": dst.name, "sha256": sha256_file(dst),
                  "size": dst.stat().st_size, **{k: v for k, v in extra.items() if v}}


def cmd_manifest(a: argparse.Namespace) -> None:
    d = Path(a.dir)
    d.mkdir(parents=True, exist_ok=True)
    arts: dict = {}
    if a.keep_existing and (d / MANIFEST_NAME).exists():
        old = parse_manifest_bytes((d / MANIFEST_NAME).read_bytes())
        for n, x in old.artifacts.items():
            verify_file(d / Path(x.url).name, x.sha256, x.size)
            arts[n] = {k: v for k, v in {"version": x.version, "url": x.url, "sha256": x.sha256,
                                         "size": x.size, "min_version": x.min_version,
                                         "platform": x.platform}.items() if v is not None}
    if a.sensor:
        _add(arts, d, "sensor", a.sensor, a.sensor_version, {"min_version": a.sensor_min})
    if a.firmware:
        _add(arts, d, "firmware", a.firmware, a.firmware_version, {"min_version": a.firmware_min})
    if a.sim:
        _add(arts, d, "sim", a.sim, a.sim_version, {"platform": a.sim_platform,
                                                    "min_version": a.sim_min})
    if not arts:
        _die("no artifacts given")
    released = a.released or datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    notes = Path(a.notes_file).read_text() if a.notes_file else (a.notes or "")
    obj = build_manifest_obj(a.channel, released, arts, notes)
    (d / MANIFEST_NAME).write_bytes(dump_manifest(obj))
    for stale in (SIG_NAME, RSASIG_NAME):
        (d / stale).unlink(missing_ok=True)   # old signatures no longer match
    print(f"wrote {d / MANIFEST_NAME} (unsigned) with {', '.join(sorted(arts))}")


# -------------------------------------------------------------------- sign
def cmd_sign(a: argparse.Namespace) -> None:
    d = Path(a.dir)
    data = (d / MANIFEST_NAME).read_bytes()
    parse_manifest_bytes(data)  # never sign something clients would reject
    kd = Path(a.key_dir) if a.key_dir else None
    ed_path = Path(a.ed25519_key) if a.ed25519_key else (kd / K.ED25519_PRIV_NAME if kd else None)
    rsa_path = Path(a.rsa_key) if a.rsa_key else (kd / K.RSA_PRIV_NAME if kd else None)
    if not ed_path:
        _die("need --ed25519-key or --key-dir")
    pw = _passphrase(a.passphrase_env, confirm=False)
    ed = K.load_private_key(ed_path, pw)
    (d / SIG_NAME).write_bytes(K.sign_ed25519(ed, data))
    print(f"wrote {d / SIG_NAME}")
    if rsa_path and rsa_path.exists():
        rs = K.load_private_key(rsa_path, pw)
        (d / RSASIG_NAME).write_bytes(K.sign_rsa(rs, data))
        print(f"wrote {d / RSASIG_NAME}")
    elif not a.no_rsa:
        _die("RSA key not found (the simulator needs manifest.json.rsasig); use --no-rsa to skip")


# ------------------------------------------------------------------ verify
def verify_release_dir(d: Path, ed_pubs: list[Path], rsa_pub: Path | None,
                       require_rsa: bool = True) -> list[str]:
    """Full offline verification of a release directory. Returns report lines;
    raises on any failure."""
    lines = []
    data = (d / MANIFEST_NAME).read_bytes()
    m = verify_manifest(data, (d / SIG_NAME).read_bytes(), K.load_ed25519_public_keys(ed_pubs))
    lines.append(f"Ed25519 signature OK ({m.channel}, released {m.released})")
    rs_path = d / RSASIG_NAME
    if rsa_pub:
        if not rs_path.exists():
            if require_rsa:
                raise ValueError(f"{RSASIG_NAME} missing")
        else:
            key = K.load_public_key(rsa_pub)
            if not K.verify_rsa(key, data, rs_path.read_bytes()):
                raise ValueError("RSA signature verification failed")
            lines.append("RSA-4096 PKCS#1 v1.5 SHA-256 signature OK")
    for name, art in sorted(m.artifacts.items()):
        f = d / Path(art.url).name
        verify_file(f, art.sha256, art.size)
        detail = ""
        if name == "firmware":
            detail = f", {validate_uf2(f.read_bytes())} UF2 blocks"
        elif name == "sensor":
            v = _sensor_version(f)
            if v != art.version:
                raise ValueError(f"sensor bundle release.json version {v} != {art.version}")
            detail = ", bundle extracts safely"
        lines.append(f"{name} {art.version}: sha256+size OK{detail}")
    return lines


def cmd_verify(a: argparse.Namespace) -> None:
    pd = Path(a.pub_dir) if a.pub_dir else None
    ed = [Path(p) for p in a.ed25519_pub] or ([pd / K.ED25519_PUB_NAME] if pd else [])
    rsa = Path(a.rsa_pub) if a.rsa_pub else (pd / K.RSA_PUB_NAME if pd else None)
    if not ed:
        _die("need --ed25519-pub or --pub-dir")
    try:
        for line in verify_release_dir(Path(a.dir), ed, rsa, require_rsa=not a.no_rsa):
            print(line)
    except Exception as e:  # noqa: BLE001
        _die(f"verification FAILED: {e}")
    print("release OK")


def cmd_serve(a: argparse.Namespace) -> None:
    from .devserver import main as serve_main
    serve_main(["--dir", a.dir, "--port", str(a.port), "--bind", a.bind])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m gs_ota.tools")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("keygen", help="generate Ed25519 + RSA-4096 release keys (offline!)")
    p.add_argument("--out", required=True)
    p.add_argument("--passphrase-env", help="read the key passphrase from this env var")
    p.add_argument("--i-know-what-im-doing", action="store_true",
                   help="allow writing keys inside a git work tree (tests only)")
    p.set_defaults(fn=cmd_keygen)

    p = sub.add_parser("build-sensor", help="build sensor-<ver>.tar.zst")
    p.add_argument("--config", default=str(DEFAULT_BUNDLE_CONFIG))
    p.add_argument("--version", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--mode", choices=["pip", "copy"])
    p.add_argument("--wheelhouse", help="offline: resolve third-party wheels only from here")
    p.add_argument("--strict", action="store_true", help="fail if any listed project is missing")
    p.set_defaults(fn=cmd_build_sensor)

    p = sub.add_parser("manifest", help="write (unsigned) manifest.json, copying artifacts in")
    p.add_argument("--dir", required=True)
    p.add_argument("--channel", default="stable")
    p.add_argument("--released", help="RFC 3339 UTC, default now")
    p.add_argument("--keep-existing", action="store_true",
                   help="carry over artifacts from the existing manifest.json in --dir")
    for n in ("sensor", "firmware", "sim"):
        p.add_argument(f"--{n}", help=f"{n} artifact file")
        p.add_argument(f"--{n}-version")
        p.add_argument(f"--{n}-min", help="min_version gate")
    p.add_argument("--sim-platform", default="windows-x86_64")
    p.add_argument("--notes")
    p.add_argument("--notes-file")
    p.set_defaults(fn=cmd_manifest)

    p = sub.add_parser("sign", help="write manifest.json.sig (+ .rsasig)")
    p.add_argument("--dir", required=True)
    p.add_argument("--key-dir")
    p.add_argument("--ed25519-key")
    p.add_argument("--rsa-key")
    p.add_argument("--no-rsa", action="store_true")
    p.add_argument("--passphrase-env")
    p.set_defaults(fn=cmd_sign)

    p = sub.add_parser("verify", help="verify a release directory completely")
    p.add_argument("--dir", required=True)
    p.add_argument("--pub-dir", help="directory with release-*.pub.pem (e.g. ota/keys)")
    p.add_argument("--ed25519-pub", action="append", default=[])
    p.add_argument("--rsa-pub")
    p.add_argument("--no-rsa", action="store_true")
    p.set_defaults(fn=cmd_verify)

    p = sub.add_parser("serve", help="LAN dev server with Range support")
    p.add_argument("--dir", default=".")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--bind", default="0.0.0.0")
    p.set_defaults(fn=cmd_serve)

    a = ap.parse_args(argv)
    a.fn(a)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
