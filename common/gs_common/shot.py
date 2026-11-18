"""Shot message, schema version 1. See docs/ARCHITECTURE.md section 4."""
from __future__ import annotations

import math
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any

SCHEMA_VERSION = 1


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


@dataclass
class BallData:
    speed_mps: float
    vla_deg: float
    hla_deg: float            # positive = right of target
    total_spin_rpm: float
    spin_axis_deg: float      # positive = curves right
    back_spin_rpm: float = float("nan")
    side_spin_rpm: float = float("nan")

    def __post_init__(self) -> None:
        if math.isnan(self.back_spin_rpm):
            self.back_spin_rpm = self.total_spin_rpm * math.cos(math.radians(self.spin_axis_deg))
        if math.isnan(self.side_spin_rpm):
            self.side_spin_rpm = self.total_spin_rpm * math.sin(math.radians(self.spin_axis_deg))


@dataclass
class ClubData:
    """Club delivery at impact. Every field except speed may be None (not measured)."""
    speed_mps: float | None = None
    path_deg: float | None = None            # horizontal club path, positive = to the right of target (in-to-out for RH)
    attack_angle_deg: float | None = None    # vertical path, negative = descending
    face_angle_deg: float | None = None      # face direction vs target line, positive = pointing right (open for RH)
    face_to_path_deg: float | None = None    # face_angle - path
    dynamic_loft_deg: float | None = None
    impact_x_mm: float | None = None         # impact point on the face from face centre, positive = toward the toe
    impact_y_mm: float | None = None         # positive = high on the face
    estimated: list[str] = field(default_factory=list)  # names of fields that are inferred, not directly measured


@dataclass
class Quality:
    confidence: float = 1.0
    sources: list[str] = field(default_factory=list)   # "stereo", "radar", "synthetic", "manual"
    spin_measured: bool = False


@dataclass
class Shot:
    ball: BallData
    club: ClubData | None = None
    quality: Quality = field(default_factory=Quality)
    shot_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    ts: str = field(default_factory=_now_iso)

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": "shot",
            "schema": SCHEMA_VERSION,
            "shot_id": self.shot_id,
            "ts": self.ts,
            "ball": asdict(self.ball),
            "club": asdict(self.club) if self.club else None,
            "quality": asdict(self.quality),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Shot":
        if d.get("schema", SCHEMA_VERSION) != SCHEMA_VERSION:
            raise ValueError(f"unsupported shot schema {d.get('schema')}")
        club = d.get("club")
        kwargs: dict[str, Any] = {}
        if "shot_id" in d:
            kwargs["shot_id"] = d["shot_id"]
        if "ts" in d:
            kwargs["ts"] = d["ts"]
        return cls(
            ball=BallData(**d["ball"]),
            club=ClubData(**club) if club else None,
            quality=Quality(**d.get("quality", {})),
            **kwargs,
        )
