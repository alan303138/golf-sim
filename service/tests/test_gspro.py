import asyncio
import json

import pytest

from gs_common import BallData, ClubData, Shot
from gs_service.gspro import GSProAdapter, shot_to_gspro


def make_shot(club=True):
    return Shot(ball=BallData(speed_mps=67.056, vla_deg=12.4, hla_deg=-1.8, total_spin_rpm=2850.0, spin_axis_deg=4.5),
                club=ClubData(speed_mps=44.704) if club else None)


def test_conversion():
    m = shot_to_gspro(make_shot().to_dict(), 3, "dev")
    assert m["APIversion"] == "1" and m["ShotNumber"] == 3 and m["DeviceID"] == "dev"
    b = m["BallData"]
    assert b["Speed"] == pytest.approx(150.0, abs=0.01)  # 67.056 m/s = 150 mph
    assert b["VLA"] == 12.4 and b["HLA"] == -1.8 and b["SpinAxis"] == 4.5
    assert b["TotalSpin"] == 2850.0 and b["BackSpin"] == pytest.approx(2841.2, abs=0.1)
    assert m["ClubData"]["Speed"] == pytest.approx(100.0, abs=0.01)
    o = m["ShotDataOptions"]
    assert o["ContainsBallData"] and o["ContainsClubData"] and not o["IsHeartBeat"]
    assert not shot_to_gspro(make_shot(False).to_dict(), 1)["ShotDataOptions"]["ContainsClubData"]


class FakeGSPro:
    def __init__(self):
        self.received: list[dict] = []
        self.connections = 0
        self.server = None
        self.writers = []

    async def start(self, port=0):
        self.server = await asyncio.start_server(self._handle, "127.0.0.1", port)
        return self.server.sockets[0].getsockname()[1]

    async def _handle(self, reader, writer):
        self.connections += 1
        self.writers.append(writer)
        dec, buf = json.JSONDecoder(), ""
        try:
            while True:
                data = await reader.read(4096)
                if not data:
                    return
                buf += data.decode()
                while buf.strip():
                    try:
                        obj, end = dec.raw_decode(buf.lstrip())
                    except json.JSONDecodeError:
                        break
                    buf = buf.lstrip()[end:]
                    self.received.append(obj)
                    writer.write(json.dumps({"Code": 200, "Message": "Shot received successfully",
                                             "Player": {"Handed": "RH", "Club": "DR"}}).encode())
                    await writer.drain()
        except ConnectionError:
            pass

    async def stop(self):
        for w in self.writers:
            w.close()
        self.server.close()
        await self.server.wait_closed()


async def wait_for(cond, t=5.0):
    for _ in range(int(t / 0.02)):
        if cond():
            return
        await asyncio.sleep(0.02)
    raise AssertionError("timeout")


async def test_adapter_sends_and_reads_response():
    fake = FakeGSPro()
    port = await fake.start()
    ad = GSProAdapter("127.0.0.1", port, reconnect_min=0.05, reconnect_max=0.1)
    await ad.start()
    try:
        await wait_for(lambda: ad.connected)
        assert await ad.send_shot(make_shot().to_dict())
        await wait_for(lambda: fake.received and ad.last_response)
        assert fake.received[0]["BallData"]["Speed"] == pytest.approx(150.0, abs=0.01)
        assert ad.last_response["Code"] == 200 and ad.last_response["Player"]["Handed"] == "RH"
        assert ad.shot_number == 1 and ad.sent == 1
    finally:
        await ad.stop()
        await fake.stop()


async def test_adapter_reconnects_and_drops_while_down():
    fake = FakeGSPro()
    port = await fake.start()
    ad = GSProAdapter("127.0.0.1", port, reconnect_min=0.05, reconnect_max=0.1)
    await ad.start()
    try:
        await wait_for(lambda: ad.connected)
        await fake.stop()
        await wait_for(lambda: not ad.connected)
        assert not await ad.send_shot(make_shot().to_dict())
        assert ad.dropped == 1
        fake2 = FakeGSPro()
        await fake2.start(port)
        try:
            await wait_for(lambda: ad.connected)
            assert await ad.send_shot(make_shot().to_dict())
            await wait_for(lambda: len(fake2.received) == 1)
        finally:
            await fake2.stop()
    finally:
        await ad.stop()


async def test_heartbeat():
    fake = FakeGSPro()
    port = await fake.start()
    ad = GSProAdapter("127.0.0.1", port, reconnect_min=0.05, heartbeat_s=0.05, status_cb=lambda: (True, False))
    await ad.start()
    try:
        await wait_for(lambda: fake.received)
        o = fake.received[0]["ShotDataOptions"]
        assert o["IsHeartBeat"] and o["LaunchMonitorIsReady"] and not o["ContainsBallData"]
    finally:
        await ad.stop()
        await fake.stop()


def test_service_forwards_shots_to_gspro(settings):
    """End-to-end: enabling GSPro via config makes injected shots arrive at the fake server."""
    import threading
    from fastapi.testclient import TestClient
    from gs_service.app import create_app

    fake = FakeGSPro()
    loop = asyncio.new_event_loop()
    t = threading.Thread(target=loop.run_forever, daemon=True)
    t.start()
    port = asyncio.run_coroutine_threadsafe(fake.start(), loop).result(5)
    try:
        with TestClient(create_app(settings)) as c:
            assert c.put("/api/v1/config", json={"gspro_enabled": True, "gspro_port": port}).status_code == 200
            import time
            for _ in range(100):
                if c.get("/api/v1/status").json()["gspro"].get("connected"):
                    break
                time.sleep(0.05)
            c.post("/api/v1/sim/shot", json={"ball": {"speed_mps": 67.056, "vla_deg": 10, "hla_deg": 0,
                                                       "total_spin_rpm": 2500, "spin_axis_deg": 0}})
            for _ in range(100):
                if fake.received:
                    break
                time.sleep(0.05)
            shots = [m for m in fake.received if m["ShotDataOptions"]["ContainsBallData"]]
            assert shots and shots[0]["BallData"]["Speed"] == pytest.approx(150.0, abs=0.01)
    finally:
        asyncio.run_coroutine_threadsafe(fake.stop(), loop).result(5)
        loop.call_soon_threadsafe(loop.stop)
