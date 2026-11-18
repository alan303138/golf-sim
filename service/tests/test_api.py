import jsonschema
import pytest
from fastapi.testclient import TestClient

from conftest import SCHEMA
from gs_service.app import create_app

BALL = {"speed_mps": 60.0, "vla_deg": 14.0, "hla_deg": -2.0, "total_spin_rpm": 3000.0, "spin_axis_deg": 10.0}


def test_status(client):
    r = client.get("/api/v1/status")
    assert r.status_code == 200
    j = r.json()
    assert j["device_id"] == "testdev" and j["api"] == 1
    assert j["state"] in ("idle", "ready", "processing", "error", "updating")
    assert "camera_left" in j["health"]


def test_cors(client):
    r = client.get("/api/v1/status", headers={"Origin": "http://example.com"})
    assert r.headers["access-control-allow-origin"] == "*"


def test_sim_shot_matches_schema_and_history(client):
    r = client.post("/api/v1/sim/shot", json={"ball": BALL, "club": {"speed_mps": 42.0}})
    assert r.status_code == 200
    shot = r.json()
    jsonschema.validate(shot, SCHEMA)
    assert shot["ball"]["back_spin_rpm"] == pytest.approx(3000 * 0.98480775, rel=1e-4)
    assert client.get(f"/api/v1/shots/{shot['shot_id']}").json() == shot
    lst = client.get("/api/v1/shots").json()
    assert lst["total"] == 1 and lst["shots"][0]["shot_id"] == shot["shot_id"]


def test_sim_shot_bare_ball_and_random(client):
    assert client.post("/api/v1/sim/shot", json=BALL).status_code == 200
    r = client.post("/api/v1/sim/shot")
    assert r.status_code == 200
    jsonschema.validate(r.json(), SCHEMA)
    assert client.post("/api/v1/sim/shot", json={"ball": {"speed_mps": 1}}).status_code == 422
    assert client.post("/api/v1/sim/shot", json={"ball": {**BALL, "speed_mps": -3}}).status_code == 422


def test_history_order_paging_delete(client):
    ids = [client.post("/api/v1/sim/shot", json={"ball": {**BALL, "speed_mps": 40 + i}}).json()["shot_id"]
           for i in range(5)]
    got = client.get("/api/v1/shots?limit=2").json()
    assert [s["shot_id"] for s in got["shots"]] == [ids[4], ids[3]]
    assert got["next_before"] == ids[3]
    nxt = client.get(f"/api/v1/shots?limit=10&before={got['next_before']}").json()
    assert [s["shot_id"] for s in nxt["shots"]] == [ids[2], ids[1], ids[0]]
    assert client.delete(f"/api/v1/shots/{ids[2]}").status_code == 200
    assert client.delete(f"/api/v1/shots/{ids[2]}").status_code == 404
    assert client.get(f"/api/v1/shots/{ids[2]}").status_code == 404
    assert client.get("/api/v1/shots").json()["total"] == 4


def test_history_persists_across_restart(settings):
    with TestClient(create_app(settings)) as c:
        sid = c.post("/api/v1/sim/shot", json={"ball": BALL}).json()["shot_id"]
    with TestClient(create_app(settings)) as c:
        assert c.get(f"/api/v1/shots/{sid}").status_code == 200


def test_config_roundtrip_and_persist(settings):
    with TestClient(create_app(settings)) as c:
        d = c.get("/api/v1/config").json()
        assert d["distance_unit"] == "yd" and d["handedness"] == "right"
        r = c.put("/api/v1/config", json={"distance_unit": "m", "speed_unit": "kmh", "altitude_m": 1200})
        assert r.status_code == 200 and r.json()["distance_unit"] == "m"
        assert c.put("/api/v1/config", json={"handedness": "x"}).status_code == 422
        assert c.put("/api/v1/config", json={"bogus": 1}).status_code == 422
    with TestClient(create_app(settings)) as c:
        d = c.get("/api/v1/config").json()
        assert d["distance_unit"] == "m" and d["altitude_m"] == 1200 and d["handedness"] == "right"


def test_update_flow(client):
    u = client.get("/api/v1/update").json()
    assert u["state"] == "idle"
    u = client.post("/api/v1/update/check").json()
    assert u["state"] == "available" and u["available"]["sensor"] == "0.2.0"
    assert client.post("/api/v1/update/apply").status_code == 202
    import time
    for _ in range(100):
        u = client.get("/api/v1/update").json()
        if u["state"] == "up_to_date":
            break
        time.sleep(0.1)
    assert u["state"] == "up_to_date" and u["current"]["sensor"] == "0.2.0"


def test_arm_disarm_rest(client):
    assert client.post("/api/v1/arm").json()["state"] == "ready"
    assert client.post("/api/v1/disarm").json()["state"] == "idle"


def test_pwa_served(client):
    r = client.get("/")
    assert r.status_code == 200 and "<html" in r.text.lower()
    assert client.get("/manifest.webmanifest").status_code == 200
    assert client.get("/sw.js").status_code == 200
    assert client.get("/api/v1/nope").status_code == 404
