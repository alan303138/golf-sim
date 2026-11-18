"""Exceptions raised by the MCU drivers."""
from __future__ import annotations

ERROR_NAMES = {
    0: "OK", 1: "UNKNOWN_CMD", 2: "BAD_LENGTH", 3: "BAD_VALUE", 4: "LIMIT_WIDTH",
    5: "LIMIT_DUTY", 6: "LIMIT_PPS", 7: "LIMIT_TIMING", 8: "BUSY_ARMED", 9: "NOT_ARMED",
    10: "FAULT_LATCHED", 11: "LOCKOUT", 12: "BAD_MAGIC", 13: "HW",
}
(OK, UNKNOWN_CMD, BAD_LENGTH, BAD_VALUE, LIMIT_WIDTH, LIMIT_DUTY, LIMIT_PPS, LIMIT_TIMING,
 BUSY_ARMED, NOT_ARMED, FAULT_LATCHED, LOCKOUT, BAD_MAGIC, HW) = range(14)

FIELD_NAMES = {
    0: "none", 1: "mode", 2: "pulses_per_exposure", 3: "burst_exposures", 4: "strobe_width_us",
    5: "strobe_interval_us", 6: "first_pulse_delay_us", 7: "exposure_tail_us", 8: "frame_period_us",
    9: "trigger_delay_us", 10: "exposure length", 11: "lockout_us", 12: "polarity",
    13: "vdiv ratio", 14: "status_period_ms",
}
_FIELD_DETAIL = {BAD_VALUE, LIMIT_TIMING}

FAULT_NAMES = {
    1 << 0: "STROBE_OVERWIDTH", 1 << 1: "DUTY_RUNTIME", 1 << 2: "PPS_RUNTIME",
    1 << 3: "INTERLOCK_TRIPPED", 1 << 4: "CLOCK", 1 << 5: "WDT_RESET",
    1 << 6: "TRIG_QUEUE", 1 << 7: "FIRE_TIMEOUT",
}


def fault_names(mask: int) -> list[str]:
    return [name for bit, name in FAULT_NAMES.items() if mask & bit]


class McuError(Exception):
    """The MCU (or the mock) rejected a command. Never a silent clip."""

    def __init__(self, code: int, detail: int = 0, request: str = ""):
        self.code = code
        self.detail = detail
        self.name = ERROR_NAMES.get(code, f"UNKNOWN({code})")
        if code in _FIELD_DETAIL:
            what = f"field {FIELD_NAMES.get(detail, detail)}"
        elif code == LIMIT_WIDTH:
            what = f"width {detail} us"
        elif code == LIMIT_DUTY:
            what = f"worst-case duty {detail} ppm"
        elif code == LIMIT_PPS:
            what = f"worst-case {detail} pulses per second"
        elif code == FAULT_LATCHED:
            what = "faults " + ",".join(fault_names(detail))
        else:
            what = f"detail {detail}"
        super().__init__(f"{request + ': ' if request else ''}{self.name} ({what})")


class McuTimeout(Exception):
    """No answer from the MCU within the timeout."""


class McuProtocolError(Exception):
    """Unexpected or incompatible protocol data."""


class McuNotFound(Exception):
    """No Pico timing MCU serial port found."""
