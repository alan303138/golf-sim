"""Cross-project interfaces. See docs/ARCHITECTURE.md section 6."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, AsyncIterator, Callable

from .shot import Shot


# ---------------------------------------------------------------- ShotSource
class DeviceState(str, Enum):
    IDLE = "idle"              # disarmed
    READY = "ready"            # armed, waiting for a shot
    PROCESSING = "processing"  # shot captured, computing
    ERROR = "error"
    UPDATING = "updating"


@dataclass
class DeviceStatus:
    state: DeviceState = DeviceState.IDLE
    ball_detected: bool = False
    message: str = ""
    health: dict[str, Any] = field(default_factory=dict)  # per-sensor details


class ShotSource(ABC):
    """Produces shots. Implemented by gs_core, consumed by gs_service."""

    @abstractmethod
    async def start(self) -> None: ...

    @abstractmethod
    async def stop(self) -> None: ...

    @abstractmethod
    async def arm(self) -> None: ...

    @abstractmethod
    async def disarm(self) -> None: ...

    @abstractmethod
    def status(self) -> DeviceStatus: ...

    @abstractmethod
    def shots(self) -> AsyncIterator[Shot]:
        """Async iterator yielding each shot as it is measured."""

    def apply_config(self, config: dict[str, Any]) -> None:
        """User settings changed (see GET/PUT /api/v1/config). Optional."""

    def fw_version(self) -> str | None:
        """Timing-MCU firmware version, if known."""
        return None


# ------------------------------------------------------------------- McuLink
@dataclass
class StrobeConfig:
    """Timing for the Pico timing MCU. The IR strobe pulse train is the time base.

    One *exposure* = the camera trigger line held active (exposure is set by the
    pulse width on IMX296) while `pulses_per_exposure` strobe pulses fire.
    mode "burst":    idle until a shot trigger (hardware TRIG_IN or fire()), then run
                     `burst_exposures` exposures back to back.
    mode "free_run": repeat exposures every `frame_period_us` continuously.
    """
    mode: str = "burst"
    strobe_width_us: int = 10          # hard-limited by firmware
    strobe_interval_us: int = 700      # between pulse starts inside one exposure
    pulses_per_exposure: int = 5
    first_pulse_delay_us: int = 50     # exposure start -> first pulse
    exposure_tail_us: int = 50         # last pulse end -> exposure end
    burst_exposures: int = 2
    frame_period_us: int = 16_700      # exposure start to exposure start
    trigger_delay_us: int = 0          # shot trigger -> first exposure start (burst mode)


@dataclass
class ShotTrigger:
    """Reported by the MCU for every exposure sequence it ran because of a shot trigger."""
    seq: int                           # increments per trigger
    t_us: int                          # MCU timestamp of the trigger event
    source: str                        # "hardware" | "software"
    exposure_start_us: list[int] = field(default_factory=list)       # MCU timestamps, one per exposure
    pulse_offsets_us: list[int] = field(default_factory=list)        # pulse start times relative to each exposure start


@dataclass
class RadarCapture:
    """Optional radar add-on: one detection with its raw I/Q window."""
    t_us: int
    sample_rate_hz: int
    pre_samples: int
    iq: Any                   # numpy complex64


@dataclass
class McuInfo:
    fw_version: str
    hw_revision: int
    serial: str
    uptime_ms: int


class McuLink(ABC):
    """Driver for the Pico timing MCU. Implemented by gs_mcu, consumed by gs_core and ota.
    Methods are thread-safe and blocking with a short timeout; callbacks run on the
    driver's reader thread."""

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def info(self) -> McuInfo: ...

    @abstractmethod
    def configure(self, config: StrobeConfig) -> None: ...

    @abstractmethod
    def arm(self, armed: bool) -> None:
        """Armed: hardware shot triggers are accepted (burst) / exposures run (free_run)."""

    @abstractmethod
    def fire(self) -> None:
        """Software shot trigger: run one burst now."""

    @abstractmethod
    def on_trigger(self, callback: Callable[[ShotTrigger], None]) -> None: ...

    @abstractmethod
    def status(self) -> dict[str, Any]:
        """Health: armed, faults (e.g. strobe limiter clipped), LED rail voltage, counters."""

    @abstractmethod
    def reboot_to_bootloader(self) -> None: ...


# ------------------------------------------------------------------- Updater
@dataclass
class UpdateState:
    phase: str = "idle"   # idle|checking|available|downloading|verifying|installing|rebooting|failed|up_to_date
    progress: float = 0.0
    message: str = ""
    current: dict[str, str] = field(default_factory=dict)    # artifact -> installed version
    available: dict[str, str] = field(default_factory=dict)  # artifact -> newer version on the server
    notes: str = ""


class Updater(ABC):
    """OTA control surface. Implemented by ota, consumed by gs_service."""

    @abstractmethod
    def state(self) -> UpdateState: ...

    @abstractmethod
    async def check(self) -> UpdateState: ...

    @abstractmethod
    async def apply(self) -> None:
        """Start download+install in the background; progress via state()/on_change."""

    @abstractmethod
    def on_change(self, callback: Callable[[UpdateState], None]) -> None: ...
