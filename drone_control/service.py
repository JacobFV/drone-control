from __future__ import annotations

import argparse
import json
import mimetypes
import re
import subprocess
import sys
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from drone_control.live_video import DirectoryFrameSource, mjpeg_chunks
from drone_control.manual_control import ManualControlConfig, ManualControlSession
from drone_control.store import ControlStationStore


REPO_ROOT = Path(__file__).resolve().parents[1]


class ControlStationHandler(BaseHTTPRequestHandler):
    server: "ControlStationServer"

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/health":
            self.send_json({"ok": True})
            return
        if parsed.path == "/api/state":
            self.send_json(self.server.store.state())
            return
        if parsed.path == "/api/wifi/capabilities":
            self.send_json(wifi_capabilities())
            return
        if parsed.path == "/api/manual/status":
            self.send_json(self.server.manual_status())
            return

        match = re.fullmatch(r"/api/records/([^/]+)/mjpeg", parsed.path)
        if match:
            query = parse_qs(parsed.query)
            fps = float(query.get("fps", ["12"])[0])
            self.send_mjpeg(match.group(1), fps=max(1.0, min(30.0, fps)))
            return

        match = re.fullmatch(r"/api/blobs/([^/]+)/(.+)", parsed.path)
        if match:
            self.send_blob_file(match.group(1), match.group(2))
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/flights":
            payload = self.read_json()
            drone_id = str(payload.get("droneId") or "")
            name = str(payload.get("name") or f"Draft flight {time.strftime('%H:%M:%S')}")
            if not drone_id:
                self.send_json({"error": "droneId is required"}, status=HTTPStatus.BAD_REQUEST)
                return
            self.send_json(
                self.server.store.create_flight(
                    drone_id,
                    name,
                    mode=str(payload.get("mode") or "manual"),
                    policy=dict(payload.get("policy") or {}),
                    metadata=dict(payload.get("metadata") or {}),
                )
            )
            return

        match = re.fullmatch(r"/api/flights/([^/]+)/records", parsed.path)
        if match:
            payload = self.read_json()
            source = resolve_repo_path(str(payload.get("source") or ""))
            if source is None:
                self.send_json({"error": "source must be inside the repository"}, status=HTTPStatus.BAD_REQUEST)
                return
            record_id = self.server.store.import_record(
                match.group(1),
                str(payload.get("type") or "artifact"),
                str(payload.get("label") or source.name),
                str(payload.get("mime") or "application/octet-stream"),
                source,
            )
            self.send_json({"id": record_id})
            return

        if parsed.path == "/api/manual/arm":
            self.server.manual.arm()
            self.send_json(self.server.manual_status())
            return
        if parsed.path == "/api/manual/disarm":
            action = self.server.manual.disarm()
            self.send_json(self.server.manual_status(action))
            return
        if parsed.path == "/api/manual/heartbeat":
            self.server.manual.heartbeat()
            self.send_json(self.server.manual_status())
            return
        if parsed.path == "/api/manual/axes":
            payload = self.read_json()
            accepted = self.server.manual.set_target_axes(
                roll=payload.get("roll"),
                pitch=payload.get("pitch"),
                throttle=payload.get("throttle"),
                yaw=payload.get("yaw"),
            )
            action = self.server.manual.tick()
            self.send_json(self.server.manual_status(action) | {"accepted": accepted})
            return
        if parsed.path == "/api/manual/stop":
            action = self.server.manual.emergency_stop()
            self.send_json(self.server.manual_status(action))
            return
        if parsed.path == "/api/manual/tick":
            action = self.server.manual.tick()
            self.send_json(self.server.manual_status(action))
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def do_PATCH(self) -> None:
        parsed = urlparse(self.path)
        match = re.fullmatch(r"/api/flights/([^/]+)", parsed.path)
        if match:
            payload = self.read_json()
            result = self.server.store.update_flight(
                match.group(1),
                name=optional_str(payload, "name"),
                mode=optional_str(payload, "mode"),
                duration=optional_str(payload, "duration"),
                policy=optional_dict(payload, "policy"),
                metadata=optional_dict(payload, "metadata"),
                metrics=optional_dict(payload, "metrics"),
            )
            if result is None:
                self.send_json({"error": "flight not found"}, status=HTTPStatus.NOT_FOUND)
                return
            self.send_json(result)
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def do_OPTIONS(self) -> None:
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Access-Control-Allow-Origin", "file://")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,PATCH,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def send_json(self, payload: object, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "file://")
        self.end_headers()
        self.wfile.write(body)

    def read_json(self) -> dict[str, object]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode())

    def send_mjpeg(self, record_id: str, fps: float) -> None:
        path = self.server.store.record_path(record_id)
        if path is None or not path.is_dir():
            self.send_error(HTTPStatus.NOT_FOUND, "frame record not found")
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        source = DirectoryFrameSource(path, fps=fps)
        try:
            for chunk in mjpeg_chunks(source):
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return
        except FileNotFoundError:
            return

    def send_blob_file(self, key: str, relative: str) -> None:
        root = self.server.store.blobs.resolve(key)
        path = (root / relative).resolve()
        if not path.is_file() or root.resolve() not in path.parents:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        data = path.read_bytes()
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt: str, *args: object) -> None:
        sys.stderr.write(f"service: {self.address_string()} {fmt % args}\n")


class ControlStationServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], store: ControlStationStore) -> None:
        super().__init__(server_address, ControlStationHandler)
        self.store = store
        self.manual = ManualControlSession(ManualControlConfig())

    def manual_status(self, action: object | None = None) -> dict[str, object]:
        payload = {
            "state": self.manual.state.value,
            "armed": self.manual.armed,
            "faultReason": self.manual.fault_reason,
            "stopReason": self.manual.stop_reason,
            "current": self.manual.current_action_dict(),
        }
        if action is not None:
            payload["action"] = action_to_dict(action)
        return payload


def main() -> int:
    parser = argparse.ArgumentParser(description="Local HTTP service for the Electron drone control station.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--db", type=Path, default=REPO_ROOT / "data" / "control_station.sqlite3")
    parser.add_argument("--blob-root", type=Path, default=REPO_ROOT / "data" / "blobs")
    args = parser.parse_args()

    store = ControlStationStore(args.db, args.blob_root, REPO_ROOT)
    store.seed_if_empty()
    server = ControlStationServer((args.host, args.port), store)
    host, port = server.server_address
    print(f"SERVICE_READY http://{host}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        store.close()
        server.server_close()
    return 0


def wifi_capabilities() -> dict[str, object]:
    try:
        iw_dev = subprocess.check_output(["iw", "dev"], text=True, stderr=subprocess.STDOUT)
        iw_list = subprocess.check_output(["iw", "list"], text=True, stderr=subprocess.STDOUT)
    except (OSError, subprocess.CalledProcessError) as exc:
        return {"available": False, "error": str(exc)}

    combinations = extract_interface_combinations(iw_list)
    supports_two_managed = any("managed" in combo and "<= 2" in combo and "#channels <= 2" in combo for combo in combinations)
    return {
        "available": True,
        "current": iw_dev,
        "validInterfaceCombinations": combinations,
        "simultaneousManagedLikely": supports_two_managed,
        "recommendation": (
            "This adapter advertises two managed/P2P-client interfaces across two channels. "
            "Use a virtual managed interface or a second USB Wi-Fi adapter; verify under NetworkManager before relying on it."
            if supports_two_managed
            else "Use a second Wi-Fi adapter or wired internet while the main radio is connected to the drone AP."
        ),
    }


def extract_interface_combinations(iw_list: str) -> list[str]:
    lines = iw_list.splitlines()
    combos: list[str] = []
    capture = False
    current: list[str] = []
    for line in lines:
        stripped = line.strip()
        if stripped == "valid interface combinations:":
            capture = True
            continue
        if not capture:
            continue
        if not stripped:
            continue
        if stripped == "HT Capability overrides:" or stripped.startswith("Device supports"):
            break
        if not line.startswith("\t"):
            break
        if stripped.startswith("* "):
            if current:
                combos.append(" ".join(current))
            current = [stripped[2:]]
        elif current:
            current.append(stripped)
    if current:
        combos.append(" ".join(current))
    return combos


def optional_str(payload: dict[str, object], key: str) -> str | None:
    if key not in payload:
        return None
    return str(payload[key])


def optional_dict(payload: dict[str, object], key: str) -> dict[str, object] | None:
    value = payload.get(key)
    return dict(value) if isinstance(value, dict) else None


def resolve_repo_path(value: str) -> Path | None:
    if not value:
        return None
    path = Path(value)
    if not path.is_absolute():
        path = REPO_ROOT / path
    resolved = path.resolve()
    try:
        resolved.relative_to(REPO_ROOT)
    except ValueError:
        return None
    return resolved


def action_to_dict(action: object) -> dict[str, object]:
    if hasattr(action, "sanitized"):
        sanitized = action.sanitized()
        return {
            "roll": sanitized.roll,
            "pitch": sanitized.pitch,
            "throttle": sanitized.throttle,
            "yaw": sanitized.yaw,
            "takeoff": sanitized.takeoff,
            "land": sanitized.land,
            "emergency_stop": sanitized.emergency_stop,
            "calibrate": sanitized.calibrate,
            "headless": sanitized.headless,
            "flip": sanitized.flip,
        }
    return {}


if __name__ == "__main__":
    raise SystemExit(main())
