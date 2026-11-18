import json

import pytest

from gs_ota import keys as K
from gs_ota.manifest import (ManifestError, SignatureError, build_manifest_obj, dump_manifest,
                             parse_manifest_bytes, verify_manifest)
from gs_ota.semver import Version, VersionError, compare

SHA = "a" * 64


def _manifest(**over):
    obj = build_manifest_obj("stable", "2026-11-16T00:00:00Z", {
        "sensor": {"version": "1.2.0", "url": "sensor-1.2.0.tar.zst", "sha256": SHA, "size": 10,
                   "min_version": "1.0.0"},
        "firmware": {"version": "1.1.0", "url": "timing-mcu-1.1.0.uf2", "sha256": SHA, "size": 5},
        "sim": {"version": "1.3.0", "url": "sim-1.3.0.pck", "sha256": SHA, "size": 7,
                "platform": "windows-x86_64"}}, "Release notes")
    obj.update(over)
    return obj


def test_semver_precedence():
    order = ["0.9.9", "1.0.0-alpha", "1.0.0-alpha.1", "1.0.0-alpha.beta", "1.0.0-beta",
             "1.0.0-beta.2", "1.0.0-beta.11", "1.0.0-rc.1", "1.0.0", "1.0.1", "1.10.0", "2.0.0"]
    vs = [Version.parse(v) for v in order]
    assert vs == sorted(vs)
    assert compare("1.0.0+build.5", "1.0.0") == 0
    for bad in ("1.0", "01.0.0", "1.0.0-", "v1.0.0", "1.0.0-01", ""):
        with pytest.raises(VersionError):
            Version.parse(bad)


def test_verify_ok_and_contract_shape(keypair):
    ed, _ = keypair
    data = dump_manifest(_manifest())
    m = verify_manifest(data, K.sign_ed25519(ed, data), [ed.public_key()])
    assert m.channel == "stable"
    assert m.artifacts["sensor"].min_version == "1.0.0"
    assert m.artifacts["sim"].platform == "windows-x86_64"
    assert m.artifact_url("sensor", "https://h/x/stable/manifest.json") == \
        "https://h/x/stable/sensor-1.2.0.tar.zst"


def test_signature_covers_every_byte(keypair):
    ed, _ = keypair
    data = dump_manifest(_manifest())
    sig = K.sign_ed25519(ed, data)
    # flip each byte position in a sample of positions, including whitespace
    for i in range(0, len(data), max(1, len(data) // 50)):
        t = bytearray(data)
        t[i] ^= 0x01
        with pytest.raises(SignatureError):
            verify_manifest(bytes(t), sig, [ed.public_key()])
    # re-serialised (semantically equal) JSON is NOT accepted: exact bytes only
    with pytest.raises(SignatureError):
        verify_manifest(json.dumps(json.loads(data)).encode(), sig, [ed.public_key()])
    with pytest.raises(SignatureError):
        verify_manifest(data + b"\n", sig, [ed.public_key()])


def test_bad_signatures(keypair):
    ed, _ = keypair
    other = K.generate_ed25519()
    data = dump_manifest(_manifest())
    sig = K.sign_ed25519(ed, data)
    with pytest.raises(SignatureError):
        verify_manifest(data, sig[:-1], [ed.public_key()])          # truncated
    with pytest.raises(SignatureError):
        verify_manifest(data, b"", [ed.public_key()])
    with pytest.raises(SignatureError):
        verify_manifest(data, K.sign_ed25519(other, data), [ed.public_key()])  # wrong key
    with pytest.raises(SignatureError):
        verify_manifest(data, sig, [])                               # no trusted keys
    # key rotation: any trusted key may sign
    assert verify_manifest(data, K.sign_ed25519(other, data), [ed.public_key(), other.public_key()])


def test_strict_parsing_after_verification(keypair):
    ed, _ = keypair
    bad = [
        b'{"schema": 1, "schema": 1}',
        dump_manifest({**_manifest(), "schema": 2}),
        dump_manifest({**_manifest(), "released": "2026-11-16 00:00:00"}),
        dump_manifest({**_manifest(), "channel": "Stable!"}),
        dump_manifest({**_manifest(), "artifacts": {"sensor": {"version": "1.2", "url": "x",
                                                               "sha256": SHA, "size": 1}}}),
        dump_manifest({**_manifest(), "artifacts": {"sensor": {"version": "1.2.0", "url": "x",
                                                               "sha256": "AB" * 32, "size": 1}}}),
        dump_manifest({**_manifest(), "artifacts": {"sensor": {"version": "1.2.0", "url": "x",
                                                               "sha256": SHA, "size": True}}}),
        dump_manifest({**_manifest(), "artifacts": {"sensor": {"version": "1.2.0", "url": "x",
                                                               "sha256": SHA, "size": 0}}}),
        b"\xff\xfe",
        b"[]",
    ]
    for data in bad:
        with pytest.raises(ManifestError):
            verify_manifest(data, K.sign_ed25519(ed, data), [ed.public_key()])
    with pytest.raises(ManifestError):
        build_manifest_obj("stable", "2026-11-16T00:00:00Z", {"sensor": {"version": "x"}})
    assert parse_manifest_bytes(dump_manifest(_manifest())).notes == "Release notes"


def test_rsa_signature_roundtrip(keypair):
    _, rs = keypair
    data = dump_manifest(_manifest())
    sig = K.sign_rsa(rs, data)
    assert len(sig) == 512
    assert K.verify_rsa(rs.public_key(), data, sig)
    assert not K.verify_rsa(rs.public_key(), data + b" ", sig)
