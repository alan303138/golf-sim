"""Lazy plugin loading from entry-point strings like ``gs_core.source:SyntheticShotSource``."""
from __future__ import annotations

import importlib
from typing import Any


def load_entry_point(spec: str) -> Any:
    """Import ``module:attr`` (attr may be dotted) and return the object."""
    if ":" not in spec:
        raise ValueError(f"entry point must look like 'package.module:Name', got {spec!r}")
    mod_name, attr = spec.split(":", 1)
    obj: Any = importlib.import_module(mod_name)
    for part in attr.split("."):
        obj = getattr(obj, part)
    return obj


def instantiate(spec: str) -> Any:
    """Load an entry point and call it with no arguments (class or factory)."""
    obj = load_entry_point(spec)
    return obj() if callable(obj) else obj


def make_source(spec: str):
    if spec in ("", "mock"):
        from .mocks import MockShotSource
        return MockShotSource()
    return instantiate(spec)


def make_updater(spec: str):
    if spec in ("", "mock"):
        from .mocks import MockUpdater
        return MockUpdater()
    return instantiate(spec)
