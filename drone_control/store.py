from __future__ import annotations

import hashlib
import json
import shutil
import sqlite3
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from drone_control.identity import drone_identity_id


SCHEMA_VERSION = 2


@dataclass(slots=True)
class BlobRef:
    key: str
    path: Path
    byte_count: int
    is_dir: bool


class BlobStore:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.root.mkdir(parents=True, exist_ok=True)

    def import_path(self, source: Path) -> BlobRef | None:
        if not source.exists():
            return None
        key = self._key_for(source)
        target = self.root / key[:2] / key
        target.parent.mkdir(parents=True, exist_ok=True)

        if source.is_dir():
            if not target.exists():
                shutil.copytree(source, target)
            return BlobRef(key=key, path=target, byte_count=directory_size(target), is_dir=True)

        if not target.exists():
            shutil.copy2(source, target)
        return BlobRef(key=key, path=target, byte_count=target.stat().st_size, is_dir=False)

    def resolve(self, key: str) -> Path:
        return self.root / key[:2] / key

    def _key_for(self, source: Path) -> str:
        digest = hashlib.sha256()
        digest.update(str(source.resolve()).encode())
        if source.is_dir():
            for path in sorted(source.rglob("*")):
                if path.is_file():
                    stat = path.stat()
                    digest.update(str(path.relative_to(source)).encode())
                    digest.update(str(stat.st_size).encode())
                    digest.update(str(int(stat.st_mtime)).encode())
        else:
            stat = source.stat()
            digest.update(str(stat.st_size).encode())
            digest.update(str(int(stat.st_mtime)).encode())
        return digest.hexdigest()


class ControlStationStore:
    """SQLite-backed store for the session-centric model.

    Hierarchy: ``environment`` (sim | real) → ``session`` (shared by all drones,
    references the environment) → ``records`` (per-drone or shared inferences).
    Drones live in their own table and are referenced by id from sessions and
    per-drone records.
    """

    def __init__(self, db_path: Path, blob_root: Path, repo_root: Path) -> None:
        self.db_path = db_path
        self.repo_root = repo_root
        self.blobs = BlobStore(blob_root)
        self.lock = threading.RLock()
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        self.conn.execute("PRAGMA foreign_keys = ON")
        self.conn.execute("PRAGMA journal_mode = WAL")
        self._init_schema()

    def close(self) -> None:
        self.conn.close()

    # ------------------------------------------------------------------ state

    def state(self) -> dict[str, Any]:
        with self.lock:
            return self._state()

    def _state(self) -> dict[str, Any]:
        environments = []
        for env in self.conn.execute("SELECT * FROM environments ORDER BY created_at DESC"):
            env_dict = {
                "id": env["id"],
                "name": env["name"],
                "kind": env["kind"],
                "config": json_loads(env["config_json"], {}),
                "createdAt": env["created_at"],
                "sessions": [],
            }
            for session in self.conn.execute(
                "SELECT * FROM sessions WHERE environment_id = ? ORDER BY created_at DESC",
                (env["id"],),
            ):
                env_dict["sessions"].append(self._session_dict(session))
            environments.append(env_dict)

        drones = []
        for drone in self.conn.execute("SELECT * FROM drones ORDER BY last_seen DESC, name"):
            drones.append(
                {
                    "id": drone["id"],
                    "name": drone["name"],
                    "model": drone["model"],
                    "status": drone["status"],
                    "lastSeen": drone["last_seen"],
                    "identity": json_loads(drone["identity_json"], {}),
                    "connection": json_loads(drone["connection_json"], {}),
                }
            )
        return {"environments": environments, "drones": drones}

    def _session_dict(self, session: sqlite3.Row) -> dict[str, Any]:
        out = {
            "id": session["id"],
            "environmentId": session["environment_id"],
            "name": session["name"],
            "state": session["state"],
            "drones": json_loads(session["drones_json"], []),
            "startedAt": session["started_at"],
            "endedAt": session["ended_at"],
            "duration": session["duration"],
            "metadata": json_loads(session["metadata_json"], {}),
            "metrics": json_loads(session["metrics_json"], {}),
            "records": [],
        }
        for record in self.conn.execute(
            "SELECT * FROM records WHERE session_id = ? ORDER BY created_at",
            (session["id"],),
        ):
            out["records"].append(self._record_dict(record))
        return out

    def _record_dict(self, record: sqlite3.Row) -> dict[str, Any]:
        item = {
            "id": record["id"],
            "sessionId": record["session_id"],
            "droneId": record["drone_id"],
            "source": record["source"],
            "type": record["type"],
            "label": record["label"],
            "mime": record["mime"],
            "byteCount": record["byte_count"],
            "metadata": json_loads(record["metadata_json"], {}),
        }
        if record["blob_key"]:
            item["blobKey"] = record["blob_key"]
            item["path"] = str(self.blobs.resolve(record["blob_key"]))
        if record["source"] == "camera":
            item["streamUrl"] = f"/api/records/{record['id']}/mjpeg"
        if record["source"] == "pose":
            item["poseUrl"] = f"/api/records/{record['id']}/pose-track"
        return item

    # ------------------------------------------------------------ environments

    def create_environment(
        self,
        name: str,
        kind: str,
        config: dict[str, Any] | None = None,
        *,
        environment_id: str | None = None,
    ) -> dict[str, Any]:
        with self.lock:
            env_id = environment_id or f"env-{uuid.uuid4().hex[:12]}"
            now = current_timestamp()
            self.conn.execute(
                """
                INSERT INTO environments (id, name, kind, config_json, created_at)
                VALUES (?, ?, ?, ?, ?)
                ON CONFLICT(id) DO UPDATE SET
                  name = excluded.name, kind = excluded.kind, config_json = excluded.config_json
                """,
                (env_id, name, kind, json.dumps(config or {}), now),
            )
            self.conn.commit()
            return {"id": env_id, "name": name, "kind": kind, "config": config or {}}

    def environment_exists(self, environment_id: str) -> bool:
        with self.lock:
            row = self.conn.execute(
                "SELECT 1 FROM environments WHERE id = ?", (environment_id,)
            ).fetchone()
            return row is not None

    # ----------------------------------------------------------------- sessions

    def create_session(
        self,
        environment_id: str,
        name: str,
        drones: list[str] | None = None,
        *,
        session_id: str | None = None,
        state: str = "recording",
        metadata: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        with self.lock:
            sid = session_id or f"session-{uuid.uuid4().hex[:12]}"
            now = current_timestamp()
            self.conn.execute(
                """
                INSERT INTO sessions
                  (id, environment_id, name, state, drones_json, started_at, ended_at,
                   duration, metadata_json, metrics_json, created_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    sid,
                    environment_id,
                    name,
                    state,
                    json.dumps(drones or []),
                    now,
                    None,
                    "00:00:00",
                    json.dumps(metadata or {}),
                    json.dumps({}),
                    now,
                ),
            )
            self.conn.commit()
            return {"id": sid, "environmentId": environment_id, "name": name, "state": state}

    def session_exists(self, session_id: str) -> bool:
        with self.lock:
            row = self.conn.execute("SELECT 1 FROM sessions WHERE id = ?", (session_id,)).fetchone()
            return row is not None

    def get_session(self, session_id: str) -> dict[str, Any] | None:
        with self.lock:
            row = self.conn.execute("SELECT * FROM sessions WHERE id = ?", (session_id,)).fetchone()
            return self._session_dict(row) if row is not None else None

    def update_session(
        self,
        session_id: str,
        *,
        name: str | None = None,
        state: str | None = None,
        drones: list[str] | None = None,
        ended_at: str | None = None,
        duration: str | None = None,
        metadata: dict[str, Any] | None = None,
        metrics: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        with self.lock:
            row = self.conn.execute("SELECT * FROM sessions WHERE id = ?", (session_id,)).fetchone()
            if row is None:
                return None
            current_metadata = json_loads(row["metadata_json"], {})
            current_metrics = json_loads(row["metrics_json"], {})
            if metadata:
                current_metadata.update(metadata)
            if metrics:
                current_metrics.update(metrics)
            self.conn.execute(
                """
                UPDATE sessions
                SET name = ?, state = ?, drones_json = ?, ended_at = ?, duration = ?,
                    metadata_json = ?, metrics_json = ?
                WHERE id = ?
                """,
                (
                    name if name is not None else row["name"],
                    state if state is not None else row["state"],
                    json.dumps(drones) if drones is not None else row["drones_json"],
                    ended_at if ended_at is not None else row["ended_at"],
                    duration if duration is not None else row["duration"],
                    json.dumps(current_metadata),
                    json.dumps(current_metrics),
                    session_id,
                ),
            )
            self.conn.commit()
            return self.get_session(session_id)

    # ------------------------------------------------------------------ records

    def import_record(
        self,
        session_id: str,
        source: str,
        record_type: str,
        label: str,
        mime: str,
        source_path: Path,
        *,
        drone_id: str | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> str:
        with self.lock:
            record_id = f"record-{uuid.uuid4().hex[:12]}"
            blob = self.blobs.import_path(source_path)
            meta = dict(metadata or {})
            if source_path.exists():
                try:
                    meta["originalPath"] = str(source_path.relative_to(self.repo_root))
                except ValueError:
                    meta["originalPath"] = str(source_path)
            else:
                meta["missingPath"] = str(source_path)
            self.conn.execute(
                """
                INSERT INTO records
                  (id, session_id, drone_id, source, type, label, mime, blob_key,
                   byte_count, metadata_json, created_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    record_id,
                    session_id,
                    drone_id,
                    source,
                    record_type,
                    label,
                    mime,
                    blob.key if blob else None,
                    blob.byte_count if blob else 0,
                    json.dumps(meta),
                    current_timestamp(),
                ),
            )
            self.conn.commit()
            return record_id

    def record_info(self, record_id: str) -> dict[str, Any] | None:
        with self.lock:
            record = self.conn.execute("SELECT * FROM records WHERE id = ?", (record_id,)).fetchone()
            if record is None:
                return None
            path = self.blobs.resolve(record["blob_key"]) if record["blob_key"] else None
            return {
                "id": record["id"],
                "sessionId": record["session_id"],
                "droneId": record["drone_id"],
                "source": record["source"],
                "type": record["type"],
                "label": record["label"],
                "mime": record["mime"],
                "blobKey": record["blob_key"],
                "path": str(path) if path else "",
                "byteCount": record["byte_count"],
                "metadata": json_loads(record["metadata_json"], {}),
            }

    def record_path(self, record_id: str) -> Path | None:
        with self.lock:
            record = self.conn.execute("SELECT blob_key FROM records WHERE id = ?", (record_id,)).fetchone()
            if not record or not record["blob_key"]:
                return None
            return self.blobs.resolve(record["blob_key"])

    def latest_record(
        self,
        session_id: str,
        *,
        source: str | None = None,
        record_type: str | None = None,
        drone_id: str | None = None,
        record_id: str | None = None,
    ) -> dict[str, Any] | None:
        """Most-recent record in a session matching the given filters."""

        with self.lock:
            clauses = ["session_id = ?"]
            params: list[Any] = [session_id]
            if source is not None:
                clauses.append("source = ?")
                params.append(source)
            if record_type is not None:
                clauses.append("type = ?")
                params.append(record_type)
            if drone_id is not None:
                clauses.append("drone_id = ?")
                params.append(drone_id)
            if record_id is not None:
                clauses.append("id = ?")
                params.append(record_id)
            query = "SELECT * FROM records WHERE " + " AND ".join(clauses) + " ORDER BY created_at DESC LIMIT 1"
            row = self.conn.execute(query, tuple(params)).fetchone()
            return self._record_dict(row) if row is not None else None

    # ------------------------------------------------------------------- drones

    def upsert_discovered_drone(
        self,
        *,
        ssid: str,
        bssid: str | None,
        iface: str,
        signal: int | None = None,
    ) -> str:
        with self.lock:
            drone_id = drone_identity_id(ssid, bssid, "ssid-scan")
            now = current_timestamp()
            identity = {
                "ssid": ssid,
                "bssid": bssid,
                "fingerprint": "provisional",
                "source": "wifi-scan",
                "confidence": "provisional-ssid" if not bssid else "bssid",
            }
            connection = {
                "ssid": ssid,
                "iface": iface,
                "ip": "192.168.1.1",
                "control": "UDP 7099",
                "camera": "RTSP 7070",
                "signal": signal,
            }
            self.conn.execute(
                """
                INSERT INTO drones
                  (id, name, model, status, last_seen, identity_json, connection_json, created_at, updated_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(id) DO UPDATE SET
                  name = excluded.name,
                  status = excluded.status,
                  last_seen = excluded.last_seen,
                  identity_json = excluded.identity_json,
                  connection_json = excluded.connection_json,
                  updated_at = excluded.updated_at
                """,
                (
                    drone_id,
                    ssid,
                    "WIFI_8K" if "8K" in ssid.upper() else "Wi-Fi drone",
                    "available",
                    now,
                    json.dumps(identity),
                    json.dumps(connection),
                    now,
                    now,
                ),
            )
            self.conn.commit()
            return drone_id

    def _insert_drone(
        self,
        *,
        drone_id: str,
        name: str,
        model: str,
        status: str,
        last_seen: str,
        identity: dict[str, Any],
        connection: dict[str, Any],
    ) -> None:
        now = current_timestamp()
        self.conn.execute(
            """
            INSERT OR IGNORE INTO drones
              (id, name, model, status, last_seen, identity_json, connection_json, created_at, updated_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (drone_id, name, model, status, last_seen, json.dumps(identity), json.dumps(connection), now, now),
        )

    # -------------------------------------------------------------------- seed

    def seed_if_empty(self) -> None:
        with self.lock:
            env_count = self.conn.execute("SELECT COUNT(*) FROM environments").fetchone()[0]
            drone_count = self.conn.execute("SELECT COUNT(*) FROM drones").fetchone()[0]
            if env_count and drone_count:
                return

            primary_drone = drone_identity_id("WIFI_8K-0c5b90", None, "ack-4802000000-rtsp-webcam")
            self._insert_drone(
                drone_id=primary_drone,
                name="WIFI_8K-0c5b90",
                model="WIFI_8K",
                status="available",
                last_seen="2026-05-12 15:09",
                identity={
                    "ssid": "WIFI_8K-0c5b90",
                    "bssid": None,
                    "controlAck": "48 02 00 00 00",
                    "rtspPath": "rtsp://192.168.1.1:7070/webcam",
                },
                connection={
                    "ssid": "WIFI_8K-0c5b90",
                    "iface": "wlP9s9",
                    "ip": "192.168.1.1",
                    "control": "UDP 7099",
                    "camera": "RTSP 7070",
                },
            )
            self._insert_drone(
                drone_id="wifi8k-second-unfingerprinted",
                name="Second WIFI_8K drone",
                model="WIFI_8K",
                status="offline",
                last_seen="2026-05-12 12:51",
                identity={"ssid": None, "bssid": None, "fingerprint": "pending"},
                connection={"ssid": "unknown", "iface": "wlP9s9", "ip": "192.168.1.1", "control": "UDP 7099", "camera": "RTSP 7070"},
            )

            if not env_count:
                self.create_environment(
                    "Live (real drones)",
                    "real",
                    {"description": "Physical WIFI_8K drones over Wi-Fi."},
                    environment_id="env-real-default",
                )
                self.create_environment(
                    "Swarm simulator",
                    "sim",
                    {"description": "Analytic-expert swarm sim with synthetic cameras."},
                    environment_id="env-sim-default",
                )
                self._seed_session(
                    environment_id="env-real-default",
                    session_id="session-20260512-150951",
                    name="Camera capture 15:09",
                    drone_id=primary_drone,
                    started_at="2026-05-12 15:09:51",
                    duration="00:00:06",
                    metadata={"battery": "fresh test battery", "location": "bench", "notes": "Autonomous RTSP camera startup verified."},
                    metrics={"packets": 1635, "bytes": 2235749, "frames": 126, "resolution": "640 x 384"},
                    records=[
                        ("camera", "frames", "Decoded forward JPEG frames", "image/jpeg-sequence", "camera_captures/frames_20260512_150951"),
                        ("artifact", "raw", "Raw UDP payloads", "application/octet-stream", "camera_captures/camera_udp_20260512_150951.bin"),
                        ("artifact", "log", "Camera session log", "text/plain", "logs/drone_camera_session_20260512_150950.log"),
                    ],
                )
                self._seed_session(
                    environment_id="env-real-default",
                    session_id="session-20260512-145202",
                    name="Phone camera sniff 14:52",
                    drone_id=primary_drone,
                    started_at="2026-05-12 14:52:02",
                    duration="00:01:00",
                    metadata={"source": "monitor pcap", "notes": "RTSP negotiation was discovered from this capture."},
                    metrics={"packets": 4114, "bytes": 5420000, "frames": 326, "resolution": "640 x 384"},
                    records=[
                        ("artifact", "pcap", "Monitor capture", "application/vnd.tcpdump.pcap", "captures/drone_monitor_20260512_145202_ch1.pcap"),
                        ("camera", "frames", "Decoded forward JPEG frames", "image/jpeg-sequence", "camera_captures/pcap_20260512_145202_jpeg_test"),
                        ("camera", "frames", "Smoothed forward JPEG frames", "image/jpeg-sequence", "camera_captures/pcap_20260512_145202_smooth_fast_test"),
                    ],
                )
            self.conn.commit()

    def _seed_session(
        self,
        *,
        environment_id: str,
        session_id: str,
        name: str,
        drone_id: str,
        started_at: str,
        duration: str,
        metadata: dict[str, Any],
        metrics: dict[str, Any],
        records: list[tuple[str, str, str, str, str]],
    ) -> None:
        now = current_timestamp()
        self.conn.execute(
            """
            INSERT INTO sessions
              (id, environment_id, name, state, drones_json, started_at, ended_at,
               duration, metadata_json, metrics_json, created_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                session_id,
                environment_id,
                name,
                "stopped",
                json.dumps([drone_id]),
                started_at,
                started_at,
                duration,
                json.dumps(metadata),
                json.dumps(metrics),
                now,
            ),
        )
        for source, record_type, label, mime, relative_path in records:
            self.import_record(
                session_id,
                source,
                record_type,
                label,
                mime,
                self.repo_root / relative_path,
                drone_id=drone_id,
            )

    # ------------------------------------------------------------------ schema

    def _init_schema(self) -> None:
        self.conn.executescript(
            """
            CREATE TABLE IF NOT EXISTS schema_meta (
              key TEXT PRIMARY KEY,
              value TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS drones (
              id TEXT PRIMARY KEY,
              name TEXT NOT NULL,
              model TEXT NOT NULL,
              status TEXT NOT NULL,
              last_seen TEXT NOT NULL,
              identity_json TEXT NOT NULL,
              connection_json TEXT NOT NULL,
              created_at TEXT NOT NULL,
              updated_at TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS environments (
              id TEXT PRIMARY KEY,
              name TEXT NOT NULL,
              kind TEXT NOT NULL,
              config_json TEXT NOT NULL,
              created_at TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS sessions (
              id TEXT PRIMARY KEY,
              environment_id TEXT NOT NULL REFERENCES environments(id) ON DELETE CASCADE,
              name TEXT NOT NULL,
              state TEXT NOT NULL,
              drones_json TEXT NOT NULL,
              started_at TEXT NOT NULL,
              ended_at TEXT,
              duration TEXT NOT NULL,
              metadata_json TEXT NOT NULL,
              metrics_json TEXT NOT NULL,
              created_at TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS records (
              id TEXT PRIMARY KEY,
              session_id TEXT NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
              drone_id TEXT,
              source TEXT NOT NULL,
              type TEXT NOT NULL,
              label TEXT NOT NULL,
              mime TEXT NOT NULL,
              blob_key TEXT,
              byte_count INTEGER NOT NULL DEFAULT 0,
              metadata_json TEXT NOT NULL,
              created_at TEXT NOT NULL
            );

            CREATE INDEX IF NOT EXISTS idx_sessions_environment_id ON sessions(environment_id);
            CREATE INDEX IF NOT EXISTS idx_records_session_id ON records(session_id);
            """
        )
        self.conn.execute(
            "INSERT OR REPLACE INTO schema_meta(key, value) VALUES('schema_version', ?)",
            (str(SCHEMA_VERSION),),
        )
        self.conn.commit()


def current_timestamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


def directory_size(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def json_loads(value: str, default: Any) -> Any:
    try:
        return json.loads(value)
    except (TypeError, json.JSONDecodeError):
        return default
