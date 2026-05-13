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
            self.send_json(self.server.store.create_flight(drone_id, name))
            return

        self.send_error(HTTPStatus.NOT_FOUND)

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
        frames = sorted(path.glob("*.jpg"))
        if not frames:
            self.send_error(HTTPStatus.NOT_FOUND, "frame record has no JPEG frames")
            return

        delay = 1.0 / fps
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        try:
            while True:
                for frame in frames:
                    data = frame.read_bytes()
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    time.sleep(delay)
        except (BrokenPipeError, ConnectionResetError):
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


if __name__ == "__main__":
    raise SystemExit(main())
