"""Command line: python -m gs_mcu [--port P | --mock] info|status|configure|arm|fire|monitor|bootloader|clear-faults|ports"""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Callable

from gs_common.interfaces import McuLink, StrobeConfig

from . import errors as E
from .mock import MockMcuLink
from .protocol import Options
from .serial_link import SerialMcuLink, find_port


def _add_config_args(p: argparse.ArgumentParser) -> None:
    d = StrobeConfig()
    p.add_argument("--mode", choices=["burst", "free_run"], default=d.mode)
    p.add_argument("--width-us", type=int, default=d.strobe_width_us, help="strobe pulse width (firmware max 20)")
    p.add_argument("--interval-us", type=int, default=d.strobe_interval_us)
    p.add_argument("--pulses", type=int, default=d.pulses_per_exposure)
    p.add_argument("--first-delay-us", type=int, default=d.first_pulse_delay_us)
    p.add_argument("--tail-us", type=int, default=d.exposure_tail_us)
    p.add_argument("--exposures", type=int, default=d.burst_exposures)
    p.add_argument("--frame-period-us", type=int, default=d.frame_period_us)
    p.add_argument("--trigger-delay-us", type=int, default=d.trigger_delay_us)


def _add_option_args(p: argparse.ArgumentParser) -> None:
    d = Options()
    p.add_argument("--cam-active", choices=["low", "high"], default="high" if d.cam_active_high else "low",
                   help="CAM_TRIG active level (default low: exposure = low pulse)")
    p.add_argument("--trig-edge", choices=["rising", "falling"], default="rising" if d.trig_rising else "falling",
                   help="TRIG_IN edge that counts as a shot (match jumper JP3)")
    p.add_argument("--lockout-us", type=int, default=d.lockout_us)
    p.add_argument("--status-period-ms", type=int, default=d.status_period_ms)
    p.add_argument("--vdiv-ratio", type=float, default=d.vdiv_ratio, help="Vrail = Vadc * ratio")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="gs_mcu", description="golf-sim timing MCU tool")
    ap.add_argument("--port", help="serial port (default: autodetect Pico 2e8a:000a, or $GS_MCU_PORT)")
    ap.add_argument("--mock", action="store_true", help="use the hardware-free MockMcuLink")
    ap.add_argument("--timeout", type=float, default=1.0)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports", help="show the port that would be used")
    sub.add_parser("info", help="firmware version, serial, limits")
    sub.add_parser("status", help="armed/faults/interlock/rail/counters")
    sub.add_parser("clear-faults")
    c = sub.add_parser("configure", help="send StrobeConfig (+ hardware options); rejected configs print the error")
    _add_config_args(c)
    _add_option_args(c)
    a = sub.add_parser("arm")
    a.add_argument("--off", action="store_true", help="disarm")
    a.add_argument("--configure", action="store_true", help="configure with the defaults first")
    sub.add_parser("fire", help="software shot trigger (the MCU must be armed)")
    m = sub.add_parser("monitor", help="arm, print triggers/events/status until Ctrl-C")
    m.add_argument("--duration", type=float, default=0.0, help="seconds (0 = until Ctrl-C)")
    m.add_argument("--no-arm", action="store_true")
    m.add_argument("--fire-every", type=float, default=0.0, help="send a software trigger every N seconds")
    _add_config_args(m)
    _add_option_args(m)
    m.add_argument("--keep-config", action="store_true", help="do not configure, use what the MCU has")
    b = sub.add_parser("bootloader", help="reboot into the USB bootloader (BOOTSEL) for flashing")
    b.add_argument("--yes", action="store_true")
    return ap


def _cfg_from(args) -> StrobeConfig:
    return StrobeConfig(mode=args.mode, strobe_width_us=args.width_us, strobe_interval_us=args.interval_us,
                        pulses_per_exposure=args.pulses, first_pulse_delay_us=args.first_delay_us,
                        exposure_tail_us=args.tail_us, burst_exposures=args.exposures,
                        frame_period_us=args.frame_period_us, trigger_delay_us=args.trigger_delay_us)


def _opts_from(args) -> Options:
    return Options(cam_active_high=args.cam_active == "high", trig_rising=args.trig_edge == "rising",
                   status_period_ms=args.status_period_ms, lockout_us=args.lockout_us, vdiv_ratio=args.vdiv_ratio)


def _print(obj) -> None:
    print(json.dumps(obj, indent=2, sort_keys=True, default=str))


def _trigger_dict(t) -> dict:
    d = {"seq": t.seq, "t_us": t.t_us, "source": t.source, "exposure_start_us": t.exposure_start_us,
         "pulse_offsets_us": t.pulse_offsets_us}
    for k in ("strobe_inhibited", "free_run"):
        if hasattr(t, k):
            d[k] = getattr(t, k)
    return d


def main(argv: list[str] | None = None, link_factory: Callable[[argparse.Namespace], McuLink] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.cmd == "ports":
        try:
            print(find_port(args.port))
            return 0
        except E.McuNotFound as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
    if link_factory is not None:
        link = link_factory(args)
    elif args.mock:
        link = MockMcuLink()
    else:
        link = SerialMcuLink(port=args.port, timeout=args.timeout)
    try:
        link.open()
    except (E.McuNotFound, E.McuTimeout, E.McuProtocolError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    try:
        return _run(args, link)
    except E.McuError as exc:
        print(f"rejected: {exc}", file=sys.stderr)
        return 1
    except (E.McuTimeout, E.McuProtocolError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 0
    finally:
        link.close()


def _run(args, link: McuLink) -> int:
    if args.cmd == "info":
        i = link.info()
        out = {"fw_version": i.fw_version, "hw_revision": i.hw_revision, "serial": i.serial, "uptime_ms": i.uptime_ms}
        m = getattr(link, "info_msg", None)
        if m is not None:
            out["protocol"] = f"{m.proto_major}.{m.proto_minor}"
            out["limits"] = {"max_width_us": m.max_width_us, "min_width_us": m.min_width_us,
                             "max_duty_ppm": m.max_duty_ppm, "duty_window_us": m.duty_window_us,
                             "max_pulses_per_second": m.max_pps, "max_pulses_per_exposure": m.max_pulses_per_exposure,
                             "max_burst_exposures": m.max_burst_exposures, "max_exposure_us": m.max_exposure_us,
                             "clock_hz": m.clk_hz}
        _print(out)
    elif args.cmd == "status":
        _print(link.status())
    elif args.cmd == "clear-faults":
        link.clear_faults()
        print("faults cleared")
    elif args.cmd == "configure":
        link.set_options(_opts_from(args))   # type: ignore[attr-defined]
        link.configure(_cfg_from(args))
        _print_report(link)
    elif args.cmd == "arm":
        if args.configure:
            link.configure(StrobeConfig())
        link.arm(not args.off)
        warn = getattr(link, "strobe_inhibited_on_arm", False)
        print("disarmed" if args.off else "armed" + (" (STROBE_EN is low: strobe inhibited)" if warn else ""))
    elif args.cmd == "fire":
        link.fire()
        print("fired")
    elif args.cmd == "bootloader":
        if not args.yes:
            print("this reboots the Pico into BOOTSEL; add --yes to confirm", file=sys.stderr)
            return 1
        link.reboot_to_bootloader()
        print("rebooting into bootloader: copy the .uf2 to the RPI-RP2 drive")
    elif args.cmd == "monitor":
        return _monitor(args, link)
    return 0


def _print_report(link) -> None:
    rep = link.get_config() if hasattr(link, "get_config") else None
    if rep is None:
        return
    _print({"accepted": True, "exposure_len_us": rep.exposure_len_us, "burst_span_us": rep.span_us,
            "lockout_effective_us": rep.lockout_eff_us, "worst_case_duty_ppm": rep.duty_ppm_worst,
            "worst_case_pulses_per_s": rep.pps_worst, "trigger_to_output_latency_ns_estimate": rep.latency_ns})


def _monitor(args, link) -> int:
    if not args.keep_config:
        link.set_options(_opts_from(args))
        link.configure(_cfg_from(args))
    link.on_trigger(lambda t: print("TRIGGER", json.dumps(_trigger_dict(t)), flush=True))
    if hasattr(link, "on_event"):
        link.on_event(lambda n, v: print(f"EVENT {n} value={v:#x}", flush=True))
    if not args.no_arm:
        link.arm(True)
    t0 = time.monotonic()
    next_status = t0
    next_fire = t0 + args.fire_every if args.fire_every else None
    try:
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            now = time.monotonic()
            if now >= next_status:
                st = link.status()
                print("STATUS", json.dumps({k: st[k] for k in ("armed", "faults", "interlock_ok", "rail_v",
                                                               "trigger_hw", "trigger_sw", "pulses_total")}), flush=True)
                next_status = now + 1.0
            if next_fire is not None and now >= next_fire:
                try:
                    link.fire()
                except E.McuError as exc:
                    print(f"fire rejected: {exc}", flush=True)
                next_fire = now + args.fire_every
            if isinstance(link, MockMcuLink) and next_fire is None:
                link.simulate_trigger()
            time.sleep(0.05)
    finally:
        if not args.no_arm:
            try:
                link.arm(False)
            except Exception:  # noqa: BLE001
                pass
    return 0
