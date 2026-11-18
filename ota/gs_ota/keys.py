"""Key generation, loading, signing and verification.

Two signature schemes cover the same manifest bytes:

* Ed25519 (``manifest.json.sig``, raw 64 bytes) -- the contract signature
  (docs/ARCHITECTURE.md section 8). Verified by the device agent.
* RSA-4096 PKCS#1 v1.5 with SHA-256 (``manifest.json.rsasig``, raw 512 bytes) --
  an additional signature the Godot simulator can check with the built-in
  ``Crypto.verify()``. See ota/SIM_UPDATE.md.

Public keys are PEM SubjectPublicKeyInfo files. Private keys are PKCS#8 PEM,
optionally passphrase-encrypted, and must never be stored in the repository.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable, Sequence

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ed25519, padding, rsa

ED25519_SIG_LEN = 64
RSA_BITS = 4096
RSA_SIG_LEN = RSA_BITS // 8

ED25519_PRIV_NAME = "release-ed25519.key"
ED25519_PUB_NAME = "release-ed25519.pub.pem"
RSA_PRIV_NAME = "release-rsa4096.key"
RSA_PUB_NAME = "release-rsa4096.pub.pem"


class KeyMaterialError(ValueError):
    """Raised for unusable key material."""


# ----------------------------------------------------------------- generation
def generate_ed25519() -> ed25519.Ed25519PrivateKey:
    return ed25519.Ed25519PrivateKey.generate()


def generate_rsa(bits: int = RSA_BITS) -> rsa.RSAPrivateKey:
    return rsa.generate_private_key(public_exponent=65537, key_size=bits)


def private_pem(key, passphrase: bytes | None = None) -> bytes:
    enc = (serialization.BestAvailableEncryption(passphrase) if passphrase
           else serialization.NoEncryption())
    return key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8, enc)


def public_pem(key) -> bytes:
    pub = key.public_key() if hasattr(key, "public_key") else key
    return pub.public_bytes(serialization.Encoding.PEM,
                            serialization.PublicFormat.SubjectPublicKeyInfo)


def write_secret(path: Path, data: bytes) -> None:
    """Write a private key with 0600 permissions, refusing to overwrite."""
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as f:
        f.write(data)


# -------------------------------------------------------------------- loading
def load_private_key(path: str | Path, passphrase: bytes | None = None):
    data = Path(path).read_bytes()
    try:
        return serialization.load_pem_private_key(data, password=passphrase)
    except TypeError as e:  # encrypted key without passphrase or vice versa
        raise KeyMaterialError(f"{path}: {e}") from e


def load_public_key(path: str | Path):
    data = Path(path).read_bytes()
    return load_public_key_bytes(data, str(path))


def load_public_key_bytes(data: bytes, label: str = "<key>"):
    text = data.strip()
    if text.startswith(b"-----BEGIN"):
        key = serialization.load_pem_public_key(text)
        if not isinstance(key, (ed25519.Ed25519PublicKey, rsa.RSAPublicKey)):
            raise KeyMaterialError(f"{label}: unsupported public key type")
        return key
    # Convenience: a raw Ed25519 key as 32 bytes or 64 hex chars.
    if len(text) == 64:
        try:
            return ed25519.Ed25519PublicKey.from_public_bytes(bytes.fromhex(text.decode()))
        except ValueError:
            pass
    if len(data) == 32:
        return ed25519.Ed25519PublicKey.from_public_bytes(data)
    raise KeyMaterialError(f"{label}: not a PEM public key or raw Ed25519 key")


def load_ed25519_public_keys(paths: Iterable[str | Path]) -> list[ed25519.Ed25519PublicKey]:
    keys = []
    for p in paths:
        k = load_public_key(p)
        if not isinstance(k, ed25519.Ed25519PublicKey):
            raise KeyMaterialError(f"{p}: expected an Ed25519 public key")
        keys.append(k)
    return keys


def key_id(pub) -> str:
    """Short fingerprint (sha256 of the SubjectPublicKeyInfo DER, 16 hex chars)."""
    import hashlib
    der = pub.public_bytes(serialization.Encoding.DER,
                           serialization.PublicFormat.SubjectPublicKeyInfo)
    return hashlib.sha256(der).hexdigest()[:16]


# ---------------------------------------------------------- sign and verify
def sign_ed25519(key: ed25519.Ed25519PrivateKey, data: bytes) -> bytes:
    if not isinstance(key, ed25519.Ed25519PrivateKey):
        raise KeyMaterialError("expected an Ed25519 private key")
    return key.sign(data)


def verify_ed25519(keys: Sequence[ed25519.Ed25519PublicKey], data: bytes, sig: bytes) -> bool:
    """True if ``sig`` is a valid signature of ``data`` by any trusted key.
    Multiple trusted keys allow key rotation."""
    if len(sig) != ED25519_SIG_LEN:
        return False
    for k in keys:
        try:
            k.verify(sig, data)
            return True
        except InvalidSignature:
            continue
    return False


def sign_rsa(key: rsa.RSAPrivateKey, data: bytes) -> bytes:
    if not isinstance(key, rsa.RSAPrivateKey):
        raise KeyMaterialError("expected an RSA private key")
    if key.key_size != RSA_BITS:
        raise KeyMaterialError(f"RSA key must be {RSA_BITS} bits, got {key.key_size}")
    return key.sign(data, padding.PKCS1v15(), hashes.SHA256())


def verify_rsa(key: rsa.RSAPublicKey, data: bytes, sig: bytes) -> bool:
    try:
        key.verify(sig, data, padding.PKCS1v15(), hashes.SHA256())
        return True
    except InvalidSignature:
        return False
