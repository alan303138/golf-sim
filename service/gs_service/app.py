"""FastAPI application factory implementing the device API (ARCHITECTURE.md section 5)."""
from __future__ import annotations

import logging
from contextlib import asynccontextmanager
from typing import Any

from fastapi import Body, FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field, ValidationError

from gs_common import BallData, ClubData, Quality, Shot, ShotSource, Updater

from . import __version__
from .mocks import MockShotSource, random_shot
from .plugins import make_source, make_updater
from .service import Service
from .settings import Settings

log = logging.getLogger("gs_service")


class BallIn(BaseModel):
    model_config = ConfigDict(extra="forbid")
    speed_mps: float = Field(ge=0, le=120)
    vla_deg: float
    hla_deg: float
    total_spin_rpm: float = Field(ge=0)
    spin_axis_deg: float
    back_spin_rpm: float | None = None
    side_spin_rpm: float | None = None


class ClubIn(BaseModel):
    speed_mps: float = Field(ge=0, le=100)


class QualityIn(BaseModel):
    confidence: float = Field(1.0, ge=0, le=1)
    sources: list[str] = ["manual"]
    spin_measured: bool = False


def create_app(settings: Settings | None = None, source: ShotSource | None = None,
               updater: Updater | None = None) -> FastAPI:
    settings = settings or Settings.from_env()
    source = source or make_source(settings.source)
    updater = updater or make_updater(settings.updater)
    svc = Service(settings, source, updater)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        await svc.start()
        try:
            yield
        finally:
            await svc.stop()

    app = FastAPI(title="Golf Sim device API", version=__version__, lifespan=lifespan)
    app.state.svc = svc
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

    # ---------------------------------------------------------------- REST
    @app.get("/api/v1/status")
    async def get_status() -> dict[str, Any]:
        return svc.status_detail()

    @app.post("/api/v1/arm")
    async def arm() -> dict[str, Any]:
        await svc.source.arm()
        return svc.status_message()

    @app.post("/api/v1/disarm")
    async def disarm() -> dict[str, Any]:
        await svc.source.disarm()
        return svc.status_message()

    @app.get("/api/v1/shots")
    async def list_shots(limit: int = Query(50, ge=1, le=1000), before: str | None = None) -> dict[str, Any]:
        shots = svc.store.list(limit, before)
        return {"shots": shots, "total": svc.store.count(),
                "next_before": shots[-1]["shot_id"] if len(shots) == limit else None}

    @app.get("/api/v1/shots/{shot_id}")
    async def get_shot(shot_id: str) -> dict[str, Any]:
        s = svc.store.get(shot_id)
        if s is None:
            raise HTTPException(404, "shot not found")
        return s

    @app.delete("/api/v1/shots/{shot_id}")
    async def delete_shot(shot_id: str) -> dict[str, Any]:
        if not svc.store.delete(shot_id):
            raise HTTPException(404, "shot not found")
        return {"deleted": shot_id}

    @app.get("/api/v1/config")
    async def get_config() -> dict[str, Any]:
        return svc.config.get()

    @app.put("/api/v1/config")
    async def put_config(patch: dict[str, Any] = Body(...)) -> dict[str, Any]:
        try:
            return svc.update_config(patch)
        except ValidationError as e:
            raise HTTPException(422, detail=e.errors(include_url=False, include_context=False))

    @app.post("/api/v1/sim/shot")
    async def sim_shot(body: dict[str, Any] | None = Body(default=None)) -> dict[str, Any]:
        """Inject a shot. Body: ``{"ball": {...}, "club": {...}?}`` (a bare ball object also works);
        an empty body generates a random plausible shot."""
        if not body:
            shot = random_shot()
        else:
            try:
                ball_d = body.get("ball", body if "speed_mps" in body else None)
                if ball_d is None:
                    raise HTTPException(422, "body needs a 'ball' object")
                ball = BallIn(**ball_d)
                club = ClubIn(**body["club"]) if body.get("club") else None
                q = QualityIn(**body["quality"]) if body.get("quality") else QualityIn()
            except ValidationError as e:
                raise HTTPException(422, detail=e.errors(include_url=False, include_context=False))
            bd = {k: v for k, v in ball.model_dump().items() if v is not None}
            shot = Shot(ball=BallData(**bd), club=ClubData(club.speed_mps) if club else None,
                        quality=Quality(**q.model_dump()))
        return await svc.ingest(shot)

    @app.get("/api/v1/update")
    async def get_update() -> dict[str, Any]:
        return svc.update_dict()

    @app.post("/api/v1/update/check")
    async def update_check() -> dict[str, Any]:
        await svc.updater.check()
        return svc.update_dict()

    @app.post("/api/v1/update/apply", status_code=202)
    async def update_apply() -> dict[str, Any]:
        await svc.updater.apply()
        return svc.update_dict()

    # ---------------------------------------------------------- WebSocket
    @app.websocket("/ws/v1")
    async def ws_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        await ws.send_json(svc.hello_message())
        await ws.send_json(svc.status_message())
        svc.clients.add(ws)
        try:
            while True:
                try:
                    msg = await ws.receive_json()
                except ValueError:
                    continue
                if isinstance(msg, dict):
                    await svc.handle_client_message(ws, msg)
        except WebSocketDisconnect:
            pass
        except Exception:
            log.debug("ws closed", exc_info=True)
        finally:
            svc.clients.discard(ws)

    # ------------------------------------------------------------- PWA
    if settings.web_dir.is_dir():
        app.mount("/", StaticFiles(directory=str(settings.web_dir), html=True), name="web")
    else:
        log.warning("web dir %s not found; PWA not served", settings.web_dir)
    return app
