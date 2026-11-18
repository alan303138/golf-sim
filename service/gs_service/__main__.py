"""``python -m gs_service`` entry point."""
from __future__ import annotations

import argparse
import logging
from pathlib import Path

import uvicorn

from .app import create_app
from .settings import Settings


def main(argv: list[str] | None = None) -> None:
    env = Settings.from_env()
    p = argparse.ArgumentParser(prog="gs_service", description="Golf simulator device API server")
    p.add_argument("--host", default=env.host)
    p.add_argument("--port", type=int, default=env.port)
    p.add_argument("--data-dir", type=Path, default=env.data_dir, help="shots.db, config.json, device_id")
    p.add_argument("--source", default=env.source, help="'mock' or entry point 'gs_core.source:SyntheticShotSource'")
    p.add_argument("--updater", default=env.updater, help="'mock' or entry point 'ota.agent:Updater'")
    p.add_argument("--web-dir", type=Path, default=env.web_dir)
    p.add_argument("--no-mdns", action="store_true", default=not env.mdns)
    p.add_argument("--gspro", action="store_true", default=env.gspro_enabled, help="enable GSPro Open Connect adapter")
    p.add_argument("--gspro-host", default=env.gspro_host)
    p.add_argument("--gspro-port", type=int, default=env.gspro_port)
    p.add_argument("--log-level", default="info")
    a = p.parse_args(argv)

    logging.basicConfig(level=a.log_level.upper(), format="%(asctime)s %(name)s %(levelname)s %(message)s")
    settings = Settings(host=a.host, port=a.port, data_dir=a.data_dir, source=a.source, updater=a.updater,
                        web_dir=a.web_dir, mdns=not a.no_mdns, device_id=env.device_id,
                        gspro_enabled=a.gspro, gspro_host=a.gspro_host, gspro_port=a.gspro_port)
    uvicorn.run(create_app(settings), host=settings.host, port=settings.port, log_level=a.log_level)


if __name__ == "__main__":
    main()
