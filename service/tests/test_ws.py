import time

import jsonschema

from conftest import SCHEMA

BALL = {"speed_mps": 60.0, "vla_deg": 14.0, "hla_deg": -2.0, "total_spin_rpm": 3000.0, "spin_axis_deg": 10.0}


def recv_until(ws, typ, limit=50):
    for _ in range(limit):
        m = ws.receive_json()
        if m["type"] == typ:
            return m
    raise AssertionError(f"no {typ} message")


def test_hello_status_shot_flow(client):
    with client.websocket_connect("/ws/v1") as ws:
        hello = ws.receive_json()
        assert hello["type"] == "hello"
        assert hello["device_id"] == "testdev" and hello["api"] == 1 and "sw_version" in hello and "fw_version" in hello
        st = ws.receive_json()
        assert st["type"] == "status"
        assert set(st) >= {"state", "ball_detected", "message"}
        sid = client.post("/api/v1/sim/shot", json={"ball": BALL}).json()["shot_id"]
        shot = recv_until(ws, "shot")
        assert shot["shot_id"] == sid
        jsonschema.validate(shot, SCHEMA)


def test_ping_arm_disarm(client):
    with client.websocket_connect("/ws/v1") as ws:
        ws.receive_json(); ws.receive_json()
        ws.send_json({"type": "ping"})
        assert recv_until(ws, "pong")["type"] == "pong"
        ws.send_json({"type": "arm"})
        for _ in range(20):
            m = recv_until(ws, "status")
            if m["state"] == "ready":
                assert m["ball_detected"] is True
                break
        else:
            raise AssertionError("never ready")
        ws.send_json({"type": "disarm"})
        for _ in range(20):
            if recv_until(ws, "status")["state"] == "idle":
                break
        else:
            raise AssertionError("never idle")


def test_periodic_status_and_update_messages(client):
    with client.websocket_connect("/ws/v1") as ws:
        ws.receive_json(); ws.receive_json()
        client.post("/api/v1/update/check")
        client.post("/api/v1/update/apply")
        phases = set()
        t0 = time.time()
        while time.time() - t0 < 10:
            m = ws.receive_json()
            if m["type"] == "update":
                assert 0 <= m["progress"] <= 1
                phases.add(m["phase"])
                if m["phase"] == "up_to_date":
                    break
        assert {"downloading", "installing", "up_to_date"} <= phases


def test_broadcast_to_multiple_clients(client):
    with client.websocket_connect("/ws/v1") as a, client.websocket_connect("/ws/v1") as b:
        for w in (a, b):
            w.receive_json(); w.receive_json()
        client.post("/api/v1/sim/shot", json={"ball": BALL})
        assert recv_until(a, "shot")["type"] == "shot"
        assert recv_until(b, "shot")["type"] == "shot"
