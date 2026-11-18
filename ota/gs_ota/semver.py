"""Semantic Versioning 2.0.0 parsing and precedence (build metadata ignored)."""
from __future__ import annotations

import re
from dataclasses import dataclass
from functools import total_ordering

_SEMVER_RE = re.compile(
    r"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)"
    r"(?:-((?:0|[1-9]\d*|\d*[a-zA-Z-][0-9a-zA-Z-]*)(?:\.(?:0|[1-9]\d*|\d*[a-zA-Z-][0-9a-zA-Z-]*))*))?"
    r"(?:\+([0-9a-zA-Z-]+(?:\.[0-9a-zA-Z-]+)*))?$"
)


class VersionError(ValueError):
    pass


@total_ordering
@dataclass(frozen=True)
class Version:
    major: int
    minor: int
    patch: int
    prerelease: tuple[str, ...] = ()
    build: str = ""

    @classmethod
    def parse(cls, text: str) -> "Version":
        if not isinstance(text, str) or len(text) > 128:
            raise VersionError(f"invalid version: {text!r}")
        m = _SEMVER_RE.match(text)
        if not m:
            raise VersionError(f"invalid semver: {text!r}")
        pre = tuple(m.group(4).split(".")) if m.group(4) else ()
        return cls(int(m.group(1)), int(m.group(2)), int(m.group(3)), pre, m.group(5) or "")

    def _key(self):
        # A version without prerelease has higher precedence than one with.
        if not self.prerelease:
            pre_key: tuple = (1,)
        else:
            parts = []
            for p in self.prerelease:
                # numeric identifiers sort before alphanumeric ones
                parts.append((0, int(p), "") if p.isdigit() else (1, 0, p))
            pre_key = (0, tuple(parts))
        return (self.major, self.minor, self.patch, pre_key)

    def __lt__(self, other: object) -> bool:
        if not isinstance(other, Version):
            return NotImplemented
        return self._key() < other._key()

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Version):
            return NotImplemented
        return self._key() == other._key()

    def __hash__(self) -> int:
        return hash(self._key())

    def __str__(self) -> str:
        s = f"{self.major}.{self.minor}.{self.patch}"
        if self.prerelease:
            s += "-" + ".".join(self.prerelease)
        if self.build:
            s += "+" + self.build
        return s


def parse(text: str) -> Version:
    return Version.parse(text)


def compare(a: str, b: str) -> int:
    va, vb = Version.parse(a), Version.parse(b)
    return (va > vb) - (va < vb)


def is_valid(text: str) -> bool:
    try:
        Version.parse(text)
        return True
    except VersionError:
        return False
