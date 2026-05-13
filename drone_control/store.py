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


SCHEMA_VERSION = 1


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

    def state(self) -> dict[str, Any]:
        with self.lock:
            return self._state()

    def _state(self) -> dict[str, Any]:
        drones = []
        for drone in self.conn.execute("SELECT * FROM drones ORDER BY last_seen DESC, name"):
            drone_dict = {
                "id": drone["id"],
                "name": drone["name"],
                "model": drone["model"],
                "status": drone["status"],
                "lastSeen": drone["last_seen"],
                "identity": json_loads(drone["identity_json"], {}),
                "connection": json_loads(drone["connection_json"], {}),
                "flights": [],
            }
            for flight in self.conn.execute(
                "SELECT * FROM flights WHERE drone_id = ? ORDER BY created_at DESC",
                (drone["id"],),
            ):
                flight_dict = {
                    "id": flight["id"],
                    "name": flight["name"],
                    "startedAt": flight["started_at"],
                    "duration": flight["duration"],
                    "mode": flight["mode"],
                    "policy": json_loads(flight["policy_json"], {}),
                    "metadata": json_loads(flight["metadata_json"], {}),
                    "metrics": json_loads(flight["metrics_json"], {}),
                    "records": [],
                }
                for record in self.conn.execute(
                    "SELECT * FROM records WHERE flight_id = ? ORDER BY created_at",
                    (flight["id"],),
                ):
                    item = {
                        "id": record["id"],
                        "type": record["type"],
                        "label": record["label"],
                        "mime": record["mime"],
                        "byteCount": record["byte_count"],
                        "metadata": json_loads(record["metadata_json"], {}),
                    }
                    if record["blob_key"]:
                        item["blobKey"] = record["blob_key"]
                        item["path"] = str(self.blobs.resolve(record["blob_key"]))
                    if record["type"] == "frames":
                        item["streamUrl"] = f"/api/records/{record['id']}/mjpeg"
                    flight_dict["records"].append(item)
                drone_dict["flights"].append(flight_dict)
            drones.append(drone_dict)
        return {"drones": drones}

    def create_flight(
        self,
        drone_id: str,
        name: str,
        mode: str = "manual",
        policy: dict[str, Any] | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        with self.lock:
            flight_id = f"flight-{uuid.uuid4().hex[:12]}"
            now = current_timestamp()
            policy_data = policy or {"name": "Manual bench test", "version": 1}
            metadata_data = metadata or {"status": "draft", "notes": "No drone IO is armed yet."}
            self.conn.execute(
                """
                INSERT INTO flights
                  (id, drone_id, name, started_at, duration, mode, policy_json, metadata_json, metrics_json, created_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    flight_id,
                    drone_id,
                    name,
                    "not started",
                    "00:00:00",
                    mode,
                    json.dumps(policy_data),
                    json.dumps(metadata_data),
                    json.dumps({"frames": 0, "packets": 0, "bytes": 0, "resolution": "pending"}),
                    now,
                ),
            )
            self.conn.commit()
            return {"id": flight_id}

    def flight_exists(self, flight_id: str) -> bool:
        with self.lock:
            row = self.conn.execute("SELECT 1 FROM flights WHERE id = ?", (flight_id,)).fetchone()
            return row is not None

    def update_flight(
        self,
        flight_id: str,
        *,
        name: str | None = None,
        mode: str | None = None,
        duration: str | None = None,
        policy: dict[str, Any] | None = None,
        metadata: dict[str, Any] | None = None,
        metrics: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        with self.lock:
            row = self.conn.execute("SELECT * FROM flights WHERE id = ?", (flight_id,)).fetchone()
            if row is None:
                return None

            current_policy = json_loads(row["policy_json"], {})
            current_metadata = json_loads(row["metadata_json"], {})
            current_metrics = json_loads(row["metrics_json"], {})
            if policy:
                current_policy.update(policy)
            if metadata:
                current_metadata.update(metadata)
            if metrics:
                current_metrics.update(metrics)

            self.conn.execute(
                """
                UPDATE flights
                SET name = ?, mode = ?, duration = ?, policy_json = ?, metadata_json = ?, metrics_json = ?
                WHERE id = ?
                """,
                (
                    name if name is not None else row["name"],
                    mode if mode is not None else row["mode"],
                    duration if duration is not None else row["duration"],
                    json.dumps(current_policy),
                    json.dumps(current_metadata),
                    json.dumps(current_metrics),
                    flight_id,
                ),
            )
            self.conn.commit()
            return {
                "id": flight_id,
                "name": name if name is not None else row["name"],
                "mode": mode if mode is not None else row["mode"],
                "duration": duration if duration is not None else row["duration"],
                "policy": current_policy,
                "metadata": current_metadata,
                "metrics": current_metrics,
            }

    def seed_if_empty(self) -> None:
        with self.lock:
            count = self.conn.execute("SELECT COUNT(*) FROM drones").fetchone()[0]
            if count:
                return

            now = current_timestamp()
            self.conn.execute(
            """
            INSERT INTO drones
              (id, name, model, status, last_seen, identity_json, connection_json, created_at, updated_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                drone_identity_id("WIFI_8K-0c5b90", None, "ack-4802000000-rtsp-webcam"),
                "WIFI_8K-0c5b90",
                "WIFI_8K",
                "available",
                "2026-05-12 15:09",
                json.dumps(
                    {
                        "ssid": "WIFI_8K-0c5b90",
                        "bssid": None,
                        "controlAck": "48 02 00 00 00",
                        "rtspPath": "rtsp://192.168.1.1:7070/webcam",
                    }
                ),
                json.dumps(
                    {
                        "ssid": "WIFI_8K-0c5b90",
                        "iface": "wlP9s9",
                        "ip": "192.168.1.1",
                        "control": "UDP 7099",
                        "camera": "RTSP 7070",
                    }
                ),
                now,
                now,
            ),
        )

            self.conn.execute(
            """
            INSERT INTO drones
              (id, name, model, status, last_seen, identity_json, connection_json, created_at, updated_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                "wifi8k-second-unfingerprinted",
                "Second WIFI_8K drone",
                "WIFI_8K",
                "offline",
                "2026-05-12 12:51",
                json.dumps({"ssid": None, "bssid": None, "fingerprint": "pending"}),
                json.dumps({"ssid": "unknown", "iface": "wlP9s9", "ip": "192.168.1.1", "control": "UDP 7099", "camera": "RTSP 7070"}),
                now,
                now,
            ),
        )

            self._seed_flight(
            drone_id=drone_identity_id("WIFI_8K-0c5b90", None, "ack-4802000000-rtsp-webcam"),
            flight_id="flight-20260512-150951",
            name="Camera capture 15:09",
            started_at="2026-05-12 15:09:51",
            duration="00:00:06",
            mode="review",
            policy={"name": "Manual camera capture", "version": 1},
            metadata={"battery": "fresh test battery", "location": "bench", "notes": "Autonomous RTSP camera startup verified."},
            metrics={"packets": 1635, "bytes": 2235749, "frames": 126, "resolution": "640 x 384", "temporalMae": 3.012, "smoothedTemporalMae": 2.145},
            records=[
                ("frames", "Decoded forward JPEG frames", "image/jpeg-sequence", "camera_captures/frames_20260512_150951"),
                ("raw", "Raw UDP payloads", "application/octet-stream", "camera_captures/camera_udp_20260512_150951.bin"),
                ("log", "Camera session log", "text/plain", "logs/drone_camera_session_20260512_150950.log"),
            ],
        )
            self._seed_flight(
            drone_id=drone_identity_id("WIFI_8K-0c5b90", None, "ack-4802000000-rtsp-webcam"),
            flight_id="flight-20260512-145202",
            name="Phone camera sniff 14:52",
            started_at="2026-05-12 14:52:02",
            duration="00:01:00",
            mode="review",
            policy={"name": "Passive monitor capture", "version": 1},
            metadata={"source": "monitor pcap", "notes": "RTSP negotiation was discovered from this capture."},
            metrics={"packets": 4114, "bytes": 5420000, "frames": 326, "resolution": "640 x 384", "temporalMae": 10.922, "smoothedTemporalMae": 7.463},
            records=[
                ("pcap", "Monitor capture", "application/vnd.tcpdump.pcap", "captures/drone_monitor_20260512_145202_ch1.pcap"),
                ("frames", "Decoded forward JPEG frames", "image/jpeg-sequence", "camera_captures/pcap_20260512_145202_jpeg_test"),
                ("frames", "Smoothed forward JPEG frames", "image/jpeg-sequence", "camera_captures/pcap_20260512_145202_smooth_fast_test"),
            ],
        )
            self.conn.commit()

    def _seed_flight(
        self,
        *,
        drone_id: str,
        flight_id: str,
        name: str,
        started_at: str,
        duration: str,
        mode: str,
        policy: dict[str, Any],
        metadata: dict[str, Any],
        metrics: dict[str, Any],
        records: list[tuple[str, str, str, str]],
    ) -> None:
        now = current_timestamp()
        self.conn.execute(
            """
            INSERT INTO flights
              (id, drone_id, name, started_at, duration, mode, policy_json, metadata_json, metrics_json, created_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                flight_id,
                drone_id,
                name,
                started_at,
                duration,
                mode,
                json.dumps(policy),
                json.dumps(metadata),
                json.dumps(metrics),
                now,
            ),
        )
        for record_type, label, mime, relative_path in records:
            self.import_record(flight_id, record_type, label, mime, self.repo_root / relative_path)

    def import_record(self, flight_id: str, record_type: str, label: str, mime: str, source: Path) -> str:
        with self.lock:
            record_id = f"record-{uuid.uuid4().hex[:12]}"
            blob = self.blobs.import_path(source)
            metadata = {"missingPath": str(source)}
            if source.exists():
                try:
                    metadata = {"originalPath": str(source.relative_to(self.repo_root))}
                except ValueError:
                    metadata = {"originalPath": str(source)}
            self.conn.execute(
                """
                INSERT INTO records
                  (id, flight_id, type, label, mime, blob_key, byte_count, metadata_json, created_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    record_id,
                    flight_id,
                    record_type,
                    label,
                    mime,
                    blob.key if blob else None,
                    blob.byte_count if blob else 0,
                    json.dumps(metadata),
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
                "flightId": record["flight_id"],
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

            CREATE TABLE IF NOT EXISTS flights (
              id TEXT PRIMARY KEY,
              drone_id TEXT NOT NULL REFERENCES drones(id) ON DELETE CASCADE,
              name TEXT NOT NULL,
              started_at TEXT NOT NULL,
              duration TEXT NOT NULL,
              mode TEXT NOT NULL,
              policy_json TEXT NOT NULL,
              metadata_json TEXT NOT NULL,
              metrics_json TEXT NOT NULL,
              created_at TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS records (
              id TEXT PRIMARY KEY,
              flight_id TEXT NOT NULL REFERENCES flights(id) ON DELETE CASCADE,
              type TEXT NOT NULL,
              label TEXT NOT NULL,
              mime TEXT NOT NULL,
              blob_key TEXT,
              byte_count INTEGER NOT NULL DEFAULT 0,
              metadata_json TEXT NOT NULL,
              created_at TEXT NOT NULL
            );

            CREATE INDEX IF NOT EXISTS idx_flights_drone_id ON flights(drone_id);
            CREATE INDEX IF NOT EXISTS idx_records_flight_id ON records(flight_id);
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
