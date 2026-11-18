"""Signed release manifest (docs/ARCHITECTURE.md section 8).

Rules implemented here:
* The Ed25519 signature is checked over the *exact* bytes of manifest.json
  before a single field is parsed or trusted (``verify_manifest``).
* After verification the JSON is validated strictly: duplicate keys, wrong
  types, malformed versions/hashes and oversize values are rejected.
"""
from __future__ import annotations

import hashlib
import json
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Sequence
from urllib.parse import urljoin, urlparse

from . import keys as _keys
from .semver import Version, VersionError

SCHEMA = 1
MAX_MANIFEST_BYTES = 256 * 1024
MAX_ARTIFACT_BYTES = 8 * 1024 ** 3
MANIFEST_NAME = "manifest.json"
SIG_NAME = "manifest.json.sig"
RSASIG_NAME = "manifest.json.rsasig"

_CHANNEL_RE = re.compile(r"^[a-z0-9][a-z0-9._-]{0,31}$")
_ARTIFACT_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_RELEASED_RE = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d{1,6})?Z$")


class ManifestError(Exception):
    """Manifest is malformed or not acceptable."""


class SignatureError(ManifestError):
    """Signature missing or invalid. Nothing in the manifest may be trusted."""


@dataclass(frozen=True)
class Artifact:
    name: str
    version: str
    url: str
    sha256: str
    size: int
    min_version: str | None = None
    platform: str | None = None

    @property
    def semver(self) -> Version:
        return Version.parse(self.version)


@dataclass(frozen=True)
class Manifest:
    schema: int
    channel: str
    released: str
    artifacts: dict[str, Artifact]
    notes: str = ""
    raw: bytes = field(default=b"", repr=False)

    @property
    def released_dt(self) -> datetime:
        return parse_released(self.released)

    @property
    def digest(self) -> str:
        return hashlib.sha256(self.raw).hexdigest()

    def artifact_url(self, name: str, manifest_url: str) -> str:
        return resolve_url(manifest_url, self.artifacts[name].url)


def parse_released(text: str) -> datetime:
    if not isinstance(text, str) or not _RELEASED_RE.match(text):
        raise ManifestError(f"'released' must be an RFC 3339 UTC timestamp ending in Z: {text!r}")
    return datetime.fromisoformat(text.replace("Z", "+00:00")).astimezone(timezone.utc)


def resolve_url(manifest_url: str, ref: str) -> str:
    """Resolve an artifact URL relative to the manifest URL (contract rule).
    Absolute http(s) URLs are tolerated; integrity comes from sha256 anyway."""
    url = urljoin(manifest_url, ref)
    scheme = urlparse(url).scheme
    if scheme not in ("http", "https", "file"):
        raise ManifestError(f"unsupported URL scheme in {ref!r}")
    return url


def _no_dupes(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for k, v in pairs:
        if k in out:
            raise ManifestError(f"duplicate key {k!r}")
        out[k] = v
    return out


def _req(d: dict, key: str, typ, where: str):
    if key not in d:
        raise ManifestError(f"{where}: missing '{key}'")
    v = d[key]
    if typ is int:
        if not isinstance(v, int) or isinstance(v, bool):
            raise ManifestError(f"{where}: '{key}' must be an integer")
    elif not isinstance(v, typ):
        raise ManifestError(f"{where}: '{key}' has wrong type")
    return v


def _semver(v: Any, where: str) -> str:
    if not isinstance(v, str):
        raise ManifestError(f"{where}: version must be a string")
    try:
        Version.parse(v)
    except VersionError as e:
        raise ManifestError(f"{where}: {e}") from None
    return v


def parse_manifest_bytes(data: bytes) -> Manifest:
    """Parse and validate. Only call this on bytes whose signature has been
    verified (or in release tooling on bytes you produced yourself)."""
    if len(data) > MAX_MANIFEST_BYTES:
        raise ManifestError("manifest too large")
    try:
        obj = json.loads(data.decode("utf-8"), object_pairs_hook=_no_dupes)
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        raise ManifestError(f"manifest is not valid UTF-8 JSON: {e}") from None
    if not isinstance(obj, dict):
        raise ManifestError("manifest must be a JSON object")
    schema = _req(obj, "schema", int, "manifest")
    if schema != SCHEMA:
        raise ManifestError(f"unsupported manifest schema {schema}")
    channel = _req(obj, "channel", str, "manifest")
    if not _CHANNEL_RE.match(channel):
        raise ManifestError(f"invalid channel {channel!r}")
    released = _req(obj, "released", str, "manifest")
    parse_released(released)
    notes = obj.get("notes", "")
    if not isinstance(notes, str) or len(notes) > 64 * 1024:
        raise ManifestError("'notes' must be a string (<= 64 KiB)")
    arts_obj = _req(obj, "artifacts", dict, "manifest")
    artifacts: dict[str, Artifact] = {}
    for name, a in arts_obj.items():
        where = f"artifacts.{name}"
        if not _ARTIFACT_NAME_RE.match(name):
            raise ManifestError(f"invalid artifact name {name!r}")
        if not isinstance(a, dict):
            raise ManifestError(f"{where} must be an object")
        version = _semver(_req(a, "version", str, where), where)
        url = _req(a, "url", str, where)
        if not url or len(url) > 1024 or any(c in url for c in "\x00\r\n\\"):
            raise ManifestError(f"{where}: invalid url")
        sha = _req(a, "sha256", str, where)
        if not _SHA256_RE.match(sha):
            raise ManifestError(f"{where}: sha256 must be 64 lowercase hex chars")
        size = _req(a, "size", int, where)
        if not 0 < size <= MAX_ARTIFACT_BYTES:
            raise ManifestError(f"{where}: size out of range")
        minv = a.get("min_version")
        if minv is not None:
            minv = _semver(minv, where + ".min_version")
        platform = a.get("platform")
        if platform is not None and (not isinstance(platform, str) or len(platform) > 64):
            raise ManifestError(f"{where}: invalid platform")
        artifacts[name] = Artifact(name, version, url, sha, size, minv, platform)
    return Manifest(schema, channel, released, artifacts, notes, bytes(data))


def verify_manifest(data: bytes, sig: bytes,
                    trusted_keys: Sequence[Any]) -> Manifest:
    """Verify the detached Ed25519 signature over the exact bytes, then parse."""
    if not trusted_keys:
        raise SignatureError("no trusted public keys configured")
    if len(data) > MAX_MANIFEST_BYTES:
        raise SignatureError("manifest too large")
    if not _keys.verify_ed25519(trusted_keys, data, sig):
        raise SignatureError("manifest signature verification failed")
    return parse_manifest_bytes(data)


def dump_manifest(obj: dict) -> bytes:
    """Canonical serialisation used by the release tools. Clients never
    re-serialise: they verify the bytes exactly as downloaded."""
    return (json.dumps(obj, indent=2, sort_keys=True, ensure_ascii=False) + "\n").encode("utf-8")


def build_manifest_obj(channel: str, released: str, artifacts: dict[str, dict],
                       notes: str = "") -> dict:
    obj = {"schema": SCHEMA, "channel": channel, "released": released,
           "artifacts": artifacts, "notes": notes}
    # validate by round trip
    parse_manifest_bytes(dump_manifest(obj))
    return obj
