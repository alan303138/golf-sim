"""Shot history in SQLite."""
from __future__ import annotations

import json
import sqlite3
import threading
from pathlib import Path
from typing import Any


class ShotStore:
    def __init__(self, path: Path | str):
        if str(path) != ":memory:":
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._db = sqlite3.connect(str(path), check_same_thread=False)
        self._lock = threading.Lock()
        with self._lock:
            self._db.execute(
                "CREATE TABLE IF NOT EXISTS shots ("
                " seq INTEGER PRIMARY KEY AUTOINCREMENT,"
                " shot_id TEXT UNIQUE NOT NULL,"
                " ts TEXT NOT NULL,"
                " data TEXT NOT NULL)"
            )
            self._db.commit()

    def add(self, shot: dict[str, Any]) -> None:
        with self._lock:
            self._db.execute(
                "INSERT OR REPLACE INTO shots (shot_id, ts, data) VALUES (?,?,?)",
                (shot["shot_id"], shot["ts"], json.dumps(shot)),
            )
            self._db.commit()

    def get(self, shot_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._db.execute("SELECT data FROM shots WHERE shot_id=?", (shot_id,)).fetchone()
        return json.loads(row[0]) if row else None

    def delete(self, shot_id: str) -> bool:
        with self._lock:
            cur = self._db.execute("DELETE FROM shots WHERE shot_id=?", (shot_id,))
            self._db.commit()
            return cur.rowcount > 0

    def count(self) -> int:
        with self._lock:
            return self._db.execute("SELECT COUNT(*) FROM shots").fetchone()[0]

    def list(self, limit: int = 50, before: str | None = None) -> list[dict[str, Any]]:
        """Newest first. ``before`` is a shot_id (exclusive) or an ISO timestamp."""
        limit = max(1, min(limit, 1000))
        with self._lock:
            if before:
                row = self._db.execute("SELECT seq FROM shots WHERE shot_id=?", (before,)).fetchone()
                if row:
                    rows = self._db.execute(
                        "SELECT data FROM shots WHERE seq<? ORDER BY seq DESC LIMIT ?", (row[0], limit)
                    ).fetchall()
                else:
                    rows = self._db.execute(
                        "SELECT data FROM shots WHERE ts<? ORDER BY seq DESC LIMIT ?", (before, limit)
                    ).fetchall()
            else:
                rows = self._db.execute("SELECT data FROM shots ORDER BY seq DESC LIMIT ?", (limit,)).fetchall()
        return [json.loads(r[0]) for r in rows]

    def close(self) -> None:
        with self._lock:
            self._db.close()
