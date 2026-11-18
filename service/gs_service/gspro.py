"""GSPro Open Connect v1 TCP adapter.

Wire format (client -> GSPro, JSON objects over a plain TCP socket):

    {"DeviceID": "...", "Units": "Yards", "ShotNumber": 1, "APIversion": "1",
     "BallData": {"Speed": mph, "SpinAxis": deg, "TotalSpin": rpm, "BackSpin": rpm,
                  "SideSpin": rpm, "HLA": deg, "VLA": deg},
     "ClubData": {"Speed": mph, ...},
     "ShotDataOptions": {"ContainsBallData": bool, "ContainsClubData": bool,
                         "LaunchMonitorIsReady": bool, "LaunchMonitorBallDetected": bool,
                         "IsHeartBeat": bool}}

GSPro answers with ``{"Code": 200, "Message": "...", "Player": {"Handed": "RH", "Club": "DR", ...}}``.
Angle sign conventions match ours (positive HLA = right, positive spin axis = curve right).
"""
from __future__ import annotations

import asyncio
import json
import logging
from typing import Any, Callable

MPS_TO_MPH = 2.2369362920544

log = logging.getLogger("gs_service.gspro")


def shot_to_gspro(shot: dict[str, Any], shot_number: int, device_id: str = "GolfSim",
                  ready: bool = True, ball_detected: bool = True) -> dict[str, Any]:
    b = shot["ball"]
    club = shot.get("club")
    club_speed = round(club["speed_mps"] * MPS_TO_MPH, 2) if club else 0.0
    return {
        "DeviceID": device_id,
        "Units": "Yards",
        "ShotNumber": shot_number,
        "APIversion": "1",
        "BallData": {
            "Speed": round(b["speed_mps"] * MPS_TO_MPH, 2),
            "SpinAxis": round(b["spin_axis_deg"], 2),
            "TotalSpin": round(b["total_spin_rpm"], 1),
            "BackSpin": round(b["back_spin_rpm"], 1),
            "SideSpin": round(b["side_spin_rpm"], 1),
            "HLA": round(b["hla_deg"], 2),
            "VLA": round(b["vla_deg"], 2),
        },
        "ClubData": {
            "Speed": club_speed,
            "AngleOfAttack": 0.0, "FaceToTarget": 0.0, "Lie": 0.0, "Loft": 0.0, "Path": 0.0,
            "SpeedAtImpact": club_speed, "VerticalFaceImpact": 0.0, "HorizontalFaceImpact": 0.0,
            "ClosureRate": 0.0,
        },
        "ShotDataOptions": {
            "ContainsBallData": True,
            "ContainsClubData": club is not None,
            "LaunchMonitorIsReady": ready,
            "LaunchMonitorBallDetected": ball_detected,
            "IsHeartBeat": False,
        },
    }


def heartbeat_message(device_id: str, ready: bool, ball_detected: bool) -> dict[str, Any]:
    return {
        "DeviceID": device_id, "Units": "Yards", "ShotNumber": 0, "APIversion": "1",
        "ShotDataOptions": {
            "ContainsBallData": False, "ContainsClubData": False,
            "LaunchMonitorIsReady": ready, "LaunchMonitorBallDetected": ball_detected,
            "IsHeartBeat": True,
        },
    }


class GSProAdapter:
    """Persistent client connection to GSPro with automatic reconnect."""

    def __init__(self, host: str = "127.0.0.1", port: int = 921, device_id: str = "GolfSim",
                 status_cb: Callable[[], tuple[bool, bool]] | None = None,
                 reconnect_min: float = 1.0, reconnect_max: float = 15.0,
                 heartbeat_s: float = 10.0, shot_number: int = 0):
        self.host, self.port, self.device_id = host, port, device_id
        self._status_cb = status_cb
        self._rmin, self._rmax, self._hb = reconnect_min, reconnect_max, heartbeat_s
        self.shot_number = shot_number
        self.connected = False
        self.sent = 0
        self.dropped = 0
        self.last_response: dict[str, Any] | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._task: asyncio.Task | None = None
        self._lock = asyncio.Lock()

    def info(self) -> dict[str, Any]:
        return {"host": self.host, "port": self.port, "connected": self.connected,
                "sent": self.sent, "dropped": self.dropped, "last_response": self.last_response}

    async def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
            self._task = None
        self._close()

    def _close(self) -> None:
        self.connected = False
        if self._writer:
            try:
                self._writer.close()
            except Exception:
                pass
            self._writer = None

    async def _run(self) -> None:
        delay = self._rmin
        while True:
            try:
                reader, writer = await asyncio.wait_for(asyncio.open_connection(self.host, self.port), 5)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                log.info("GSPro connect to %s:%s failed: %s", self.host, self.port, e)
                await asyncio.sleep(delay)
                delay = min(delay * 2, self._rmax)
                continue
            delay = self._rmin
            self._writer = writer
            self.connected = True
            log.info("GSPro connected %s:%s", self.host, self.port)
            hb = asyncio.create_task(self._heartbeat())
            try:
                await self._read_loop(reader)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                log.info("GSPro connection error: %s", e)
            finally:
                hb.cancel()
                self._close()
            await asyncio.sleep(delay)

    async def _read_loop(self, reader: asyncio.StreamReader) -> None:
        dec = json.JSONDecoder()
        buf = ""
        while True:
            chunk = await reader.read(4096)
            if not chunk:
                return
            buf += chunk.decode("utf-8", "replace")
            while True:
                buf = buf.lstrip()
                if not buf:
                    break
                try:
                    obj, end = dec.raw_decode(buf)
                except json.JSONDecodeError:
                    break  # incomplete
                buf = buf[end:]
                if isinstance(obj, dict):
                    self.last_response = obj

    async def _heartbeat(self) -> None:
        while True:
            await asyncio.sleep(self._hb)
            ready, det = self._status_cb() if self._status_cb else (True, False)
            await self._send(heartbeat_message(self.device_id, ready, det))

    async def _send(self, msg: dict[str, Any]) -> bool:
        w = self._writer
        if not self.connected or w is None:
            return False
        async with self._lock:
            try:
                w.write(json.dumps(msg).encode())
                await w.drain()
                return True
            except Exception as e:
                log.info("GSPro send failed: %s", e)
                self._close()
                return False

    async def send_shot(self, shot: dict[str, Any]) -> bool:
        self.shot_number += 1
        ready, det = self._status_cb() if self._status_cb else (True, True)
        ok = await self._send(shot_to_gspro(shot, self.shot_number, self.device_id, ready, det))
        if ok:
            self.sent += 1
        else:
            self.dropped += 1
        return ok
