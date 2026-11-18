import json
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from gs_service.app import create_app
from gs_service.settings import Settings

SCHEMA = json.loads((Path(__file__).resolve().parents[2] / "common/schema/shot.schema.json").read_text())


@pytest.fixture
def settings(tmp_path):
    return Settings(data_dir=tmp_path / "data", mdns=False, web_dir=Path(__file__).resolve().parents[1] / "web",
                    device_id="testdev")


@pytest.fixture
def client(settings):
    with TestClient(create_app(settings)) as c:
        yield c
