from __future__ import annotations

import argparse
import html
import json
import mimetypes
import os
import platform
import re
import shlex
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import asdict
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from drone_control.discovery import connect_wifi as platform_connect_wifi
from drone_control.discovery import current_wifi_connection as platform_current_wifi_connection
from drone_control.discovery import default_wifi_interface, platform_network_summary
from drone_control.discovery import reconnect_wifi as platform_reconnect_wifi
from drone_control.discovery import scan_access_points, wifi_interfaces
from drone_control.flight_session import FlightSession, FlightSessionManager
from drone_control.intrinsics import load_intrinsics
from drone_control.live_video import DirectoryFrameSource, LiveDroneFrameSource, LiveDroneFrameSourceConfig, mjpeg_chunks
from drone_control.manual_control import ManualControlConfig, ManualControlSession
from drone_control.manual_transport import ManualDroneTransport
from drone_control.pose_estimator import estimator_available, load_pose_track, replay_directory
from drone_control.reconstruction import ReconstructionManager, find_splat_artifact
from drone_control.config import load_config
from drone_control.coordinator.http_vlm import HttpVLMClient, HttpVLMConfig
from drone_control.coordinator.scheduler import CoordinatorScheduler
from drone_control.coordinator.tasks import Mission, MissionProgress
from drone_control.coordinator.vlm import VLMCoordinator
from drone_control.runtime.manager import RuntimeManager, RuntimeManagerConfig
from drone_control.sim.session import SimSession, SimSessionConfig
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
        if parsed.path == "/api/system/network":
            self.send_json(platform_network_summary() | {"previousConnections": self.server.wifi_previous})
            return
        if parsed.path == "/api/config":
            self.send_json(self.server.config_status())
            return
        if parsed.path == "/api/reconstruction/tools":
            self.send_json(self.server.reconstructions.tools_status())
            return
        if parsed.path == "/api/world/splat/status":
            self.send_json(self.server.runtime.world_model_status())
            return
        if parsed.path == "/api/guidance/status":
            self.send_json({"guidance": self.server.runtime.guidance_status()})
            return
        if parsed.path == "/api/sim/status":
            self.send_json(self.server.sim.status())
            return
        if parsed.path == "/api/sim/trajectories":
            self.send_json(self.server.sim.trajectories())
            return
        match = re.fullmatch(r"/api/sim/drones/([0-9]+)/frame", parsed.path)
        if match:
            self.send_jpeg(self.server.sim.frame(int(match.group(1))))
            return
        if parsed.path == "/api/runtime/trajectories":
            self.send_json({"drones": self.server.runtime.trajectories()})
            return
        match = re.fullmatch(r"/api/runtime/drones/([^/]+)/frame", parsed.path)
        if match:
            frame = self.server.runtime.frame_registry.latest(match.group(1))
            self.send_jpeg(frame.jpeg if frame is not None else None)
            return
        if parsed.path == "/api/world/splat/snapshot":
            self.serve_world_snapshot()
            return
        if parsed.path == "/api/wifi/capabilities":
            self.send_json(wifi_capabilities())
            return
        if parsed.path == "/api/wifi/interfaces":
            try:
                interfaces = [asdict(item) for item in wifi_interfaces()]
            except (OSError, subprocess.CalledProcessError) as exc:
                self.send_json({"interfaces": [], "error": str(exc)}, status=HTTPStatus.SERVICE_UNAVAILABLE)
                return
            self.send_json({"interfaces": interfaces})
            return
        if parsed.path == "/api/wifi/access-points":
            query = parse_qs(parsed.query)
            iface = query.get("iface", [""])[0] or None
            rescan = query.get("rescan", ["1"])[0] != "0"
            try:
                access_points = [asdict(item) for item in scan_access_points(iface, rescan=rescan)]
            except (OSError, subprocess.CalledProcessError) as exc:
                self.send_json({"accessPoints": [], "error": str(exc)}, status=HTTPStatus.SERVICE_UNAVAILABLE)
                return
            self.send_json({"accessPoints": access_points})
            return
        if parsed.path == "/api/manual/status":
            with self.server.manual_lock:
                self.send_json(self.server.manual_status())
            return
        if parsed.path == "/api/runtime/status":
            self.send_json(self.server.runtime_status())
            return
        if parsed.path == "/api/runtime/events":
            query = parse_qs(parsed.query)
            since = int(query.get("since", ["0"])[0])
            self.send_json(self.server.runtime.event_stream(since=since))
            return
        if parsed.path == "/api/mission/progress":
            self.send_json(self.server.mission_progress())
            return

        match = re.fullmatch(r"/api/flights/([^/]+)/session", parsed.path)
        if match:
            status = self.server.sessions.status(match.group(1))
            self.send_json(status.as_dict() if status else {"running": False, "flightId": match.group(1)})
            return

        match = re.fullmatch(r"/api/flights/([^/]+)/pose/status", parsed.path)
        if match:
            self.send_json(self.server.pose_status(match.group(1)))
            return

        match = re.fullmatch(r"/api/flights/([^/]+)/pose/track", parsed.path)
        if match:
            query = parse_qs(parsed.query)
            since = int(query.get("since", ["-1"])[0])
            self.send_json(self.server.pose_track(match.group(1), since=since))
            return

        match = re.fullmatch(r"/api/flights/([^/]+)/reconstruction/status", parsed.path)
        if match:
            self.send_json(self.server.reconstruction_status(match.group(1)))
            return

        match = re.fullmatch(r"/api/records/([^/]+)/mjpeg", parsed.path)
        if match:
            query = parse_qs(parsed.query)
            fps = float(query.get("fps", ["12"])[0])
            self.send_mjpeg(match.group(1), fps=max(1.0, min(30.0, fps)))
            return

        match = re.fullmatch(r"/api/records/([^/]+)/artifact", parsed.path)
        if match:
            query = parse_qs(parsed.query)
            relative = query.get("path", [""])[0]
            self.send_record_artifact(match.group(1), relative)
            return

        match = re.fullmatch(r"/api/records/([^/]+)/splat-viewer", parsed.path)
        if match:
            self.send_splat_viewer(match.group(1))
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
            with self.server.manual_lock:
                try:
                    self.server.manual.arm()
                except RuntimeError as exc:
                    self.send_json({"error": str(exc), **self.server.manual_status()}, status=HTTPStatus.CONFLICT)
                    return
                self.send_json(self.server.manual_status())
            return
        if parsed.path == "/api/manual/clear-fault":
            with self.server.manual_lock:
                self.server.manual.clear_fault()
                self.send_json(self.server.manual_status())
            return
        if parsed.path == "/api/manual/config":
            self.update_manual_config()
            return
        if parsed.path == "/api/manual/disarm":
            with self.server.manual_lock:
                action = self.server.manual.disarm()
                self.server.send_manual_action(action)
                self.send_json(self.server.manual_status(action))
            return
        if parsed.path == "/api/manual/heartbeat":
            with self.server.manual_lock:
                self.server.manual.heartbeat()
                self.send_json(self.server.manual_status())
            return
        if parsed.path == "/api/manual/axes":
            payload = self.read_json()
            with self.server.manual_lock:
                accepted = self.server.manual.set_target_axes(
                    roll=payload.get("roll"),
                    pitch=payload.get("pitch"),
                    throttle=payload.get("throttle"),
                    yaw=payload.get("yaw"),
                )
                self.send_json(self.server.manual_status() | {"accepted": accepted})
            return
        if parsed.path == "/api/manual/stop":
            with self.server.manual_lock:
                action = self.server.manual.emergency_stop()
                self.server.send_manual_action(action)
                self.send_json(self.server.manual_status(action))
            return
        if parsed.path == "/api/manual/tick":
            with self.server.manual_lock:
                action = self.server.manual.tick()
                self.server.send_manual_action(action)
                self.send_json(self.server.manual_status(action))
            return

        if parsed.path == "/api/runtime/start":
            try:
                self.server.runtime.start_all()
            except (OSError, RuntimeError, ValueError) as exc:
                self.send_json({"error": str(exc), **self.server.runtime_status()}, status=HTTPStatus.CONFLICT)
                return
            self.send_json(self.server.runtime_status())
            return
        if parsed.path == "/api/runtime/stop":
            self.server.runtime.stop_all()
            self.send_json(self.server.runtime_status())
            return

        if parsed.path == "/api/world/splat/start":
            self.send_json(self.server.runtime.start_world_model())
            return
        if parsed.path == "/api/world/splat/stop":
            self.send_json(self.server.runtime.stop_world_model())
            return
        if parsed.path == "/api/world/splat/bootstrap":
            payload = self.read_json()
            flight_drones = payload.get("flightIds") or payload.get("flights")
            if flight_drones:
                self.bootstrap_world_model(flight_drones)
                return
            transforms = payload.get("transforms") or {}
            applied = []
            for drone_id, transform in transforms.items():
                try:
                    self.server.runtime.set_world_transform(str(drone_id), transform)
                    applied.append(str(drone_id))
                except Exception as exc:
                    self.send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                    return
            self.send_json({"applied": applied} | self.server.runtime.world_model_status())
            return

        if parsed.path == "/api/runtime/controller":
            payload = self.read_json()
            try:
                self.server.runtime.set_all_controllers(str(payload.get("mode") or "disabled"))
            except (KeyError, ValueError) as exc:
                self.send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return
            self.send_json(self.server.runtime_status())
            return

        match = re.fullmatch(r"/api/runtime/drones/([^/]+)/controller", parsed.path)
        if match:
            payload = self.read_json()
            try:
                self.server.runtime.set_controller(match.group(1), str(payload.get("mode") or "disabled"))
            except (KeyError, ValueError) as exc:
                self.send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return
            self.send_json(self.server.runtime_status())
            return
        match = re.fullmatch(r"/api/runtime/drones/([^/]+)/arm", parsed.path)
        if match:
            try:
                self.server.runtime.arm(match.group(1))
            except (KeyError, RuntimeError) as exc:
                self.send_json({"error": str(exc)}, status=HTTPStatus.CONFLICT)
                return
            self.send_json(self.server.runtime_status())
            return
        match = re.fullmatch(r"/api/runtime/drones/([^/]+)/disarm", parsed.path)
        if match:
            try:
                self.server.runtime.disarm(match.group(1))
            except KeyError as exc:
                self.send_json({"error": str(exc)}, status=HTTPStatus.NOT_FOUND)
                return
            self.send_json(self.server.runtime_status())
            return
        match = re.fullmatch(r"/api/runtime/drones/([^/]+)/heartbeat", parsed.path)
        if match:
            try:
                self.server.runtime.heartbeat(match.group(1))
            except KeyError as exc:
                self.send_json({"error": str(exc)}, status=HTTPStatus.NOT_FOUND)
                return
            self.send_json(self.server.runtime_status())
            return
        match = re.fullmatch(r"/api/runtime/drones/([^/]+)/axes", parsed.path)
        if match:
            try:
                self.server.runtime.set_manual_axes(match.group(1), self.read_json())
            except KeyError as exc:
                self.send_json({"error": str(exc)}, status=HTTPStatus.NOT_FOUND)
                return
            self.send_json(self.server.runtime_status())
            return
        match = re.fullmatch(r"/api/runtime/drones/([^/]+)/stop", parsed.path)
        if match:
            try:
                self.server.runtime.stop_manual(match.group(1))
            except KeyError as exc:
                self.send_json({"error": str(exc)}, status=HTTPStatus.NOT_FOUND)
                return
            self.send_json(self.server.runtime_status())
            return
        match = re.fullmatch(r"/api/runtime/drones/([^/]+)/clear-fault", parsed.path)
        if match:
            try:
                self.server.runtime.clear_fault(match.group(1))
            except KeyError as exc:
                self.send_json({"error": str(exc)}, status=HTTPStatus.NOT_FOUND)
                return
            self.send_json(self.server.runtime_status())
            return
        if parsed.path == "/api/sim/start":
            payload = self.read_json()
            cfg = SimSessionConfig(
                num_drones=int(payload.get("numDrones") or 4),
                task=str(payload.get("task") or "goto"),
                rate_hz=float(payload.get("rateHz") or 15.0),
                render=bool(payload.get("render", True)),
            )
            try:
                self.send_json(self.server.sim.start(cfg))
            except Exception as exc:
                self.send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return
        if parsed.path == "/api/sim/stop":
            self.send_json(self.server.sim.stop())
            return

        if parsed.path == "/api/guidance/tools":
            payload = self.read_json()
            calls = payload.get("calls") or payload.get("toolCalls") or []
            results = self.server.runtime.apply_guidance_tool_calls(calls)
            self.send_json({"results": results, "guidance": self.server.runtime.guidance_status()})
            return
        match = re.fullmatch(r"/api/guidance/drones/([^/]+)", parsed.path)
        if match:
            self.set_drone_guidance(match.group(1))
            return
        match = re.fullmatch(r"/api/runtime/drones/([^/]+)/camera/start", parsed.path)
        if match:
            self.start_drone_camera(match.group(1))
            return
        match = re.fullmatch(r"/api/runtime/drones/([^/]+)/camera/stop", parsed.path)
        if match:
            self.server.runtime.detach_frame_source(match.group(1))
            self.send_json({"ingestion": self.server.runtime.ingestion_status()})
            return

        if parsed.path == "/api/mission/start":
            payload = self.read_json()
            mission_id = str(payload.get("id") or f"mission-{int(time.time())}")
            objective = str(payload.get("objective") or "civilian robotics training")
            self.server.coordinator.start(Mission(mission_id, objective, dict(payload.get("context") or {})))
            controller_mode = str(payload.get("controllerMode") or os.environ.get("DRONE_MISSION_CONTROLLER", "autonomy"))
            if payload.get("setControllers") is not False:
                self.server.runtime.set_all_controllers(controller_mode)
            if payload.get("startRuntime") is not False:
                self.server.runtime.start_all()
            self.send_json(self.server.mission_progress())
            return
        if parsed.path == "/api/mission/stop":
            self.server.coordinator.stop()
            self.send_json(self.server.mission_progress())
            return

        match = re.fullmatch(r"/api/flights/([^/]+)/session/start", parsed.path)
        if match:
            self.start_flight_session(match.group(1))
            return

        match = re.fullmatch(r"/api/flights/([^/]+)/session/stop", parsed.path)
        if match:
            self.stop_flight_session(match.group(1))
            return

        match = re.fullmatch(r"/api/flights/([^/]+)/pose/compute", parsed.path)
        if match:
            self.compute_pose_track(match.group(1))
            return

        match = re.fullmatch(r"/api/flights/([^/]+)/reconstruction/start", parsed.path)
        if match:
            self.start_reconstruction(match.group(1))
            return

        match = re.fullmatch(r"/api/flights/([^/]+)/reconstruction/stop", parsed.path)
        if match:
            self.stop_reconstruction(match.group(1))
            return

        if parsed.path == "/api/wifi/connect":
            self.connect_wifi()
            return

        if parsed.path == "/api/wifi/reconnect":
            self.reconnect_wifi()
            return

        if parsed.path == "/api/drones/discover":
            self.discover_drones()
            return

        match = re.fullmatch(r"/api/records/([^/]+)/reveal", parsed.path)
        if match:
            self.reveal_record(match.group(1))
            return

        match = re.fullmatch(r"/api/records/([^/]+)/export", parsed.path)
        if match:
            self.export_record(match.group(1))
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

    def update_manual_config(self) -> None:
        payload = self.read_json()
        with self.server.manual_lock:
            try:
                self.server.manual.configure(
                    max_throttle=optional_int(payload, "maxThrottle"),
                    command_hz=optional_float(payload, "commandHz"),
                    throttle_slew_per_second=optional_float(payload, "throttleSlewPerSecond"),
                    heartbeat_timeout_seconds=optional_float(payload, "heartbeatTimeoutSeconds"),
                )
                self.server.manual_transport.configure(
                    enabled=optional_bool(payload, "enabled"),
                    iface=optional_str(payload, "iface"),
                    ip=optional_str(payload, "ip"),
                    port=optional_int(payload, "port"),
                    protocol=optional_str(payload, "protocol"),
                    bind_device=optional_bool(payload, "bindDevice"),
                    link_type=optional_str(payload, "linkType"),
                    ssid=optional_str(payload, "ssid"),
                    password=optional_str(payload, "password"),
                    serial_port=optional_str(payload, "serialPort"),
                    serial_baud=optional_int(payload, "serialBaud"),
                    esp_connect_timeout=optional_float(payload, "espConnectTimeout"),
                )
            except (RuntimeError, ValueError) as exc:
                self.send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
                return
            self.send_json(self.server.config_status())

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

    def serve_world_snapshot(self) -> None:
        export_dir = self.server.reconstruction_root / "world_model"
        export_dir.mkdir(parents=True, exist_ok=True)
        try:
            path = self.server.runtime.export_world_model(export_dir / "world.ply")
        except Exception as exc:
            self.send_json({"error": str(exc)}, status=HTTPStatus.CONFLICT)
            return
        if path is None:
            self.send_json({"error": "world model not running"}, status=HTTPStatus.NOT_FOUND)
            return
        data = Path(path).read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "model/vnd.gaussian-splat")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(data)

    def send_jpeg(self, data: bytes | None) -> None:
        if not data:
            self.send_json({"error": "no frame"}, status=HTTPStatus.NOT_FOUND)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(data)

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

    def send_record_artifact(self, record_id: str, relative: str) -> None:
        path = self.server.store.record_path(record_id)
        if path is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if path.is_dir():
            target = (path / relative).resolve()
            if not target.is_file() or path.resolve() not in target.parents:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
        else:
            if relative:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            target = path
        data = target.read_bytes()
        content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "public, max-age=3600")
        self.end_headers()
        self.wfile.write(data)

    def send_splat_viewer(self, record_id: str) -> None:
        path = self.server.store.record_path(record_id)
        info = self.server.store.record_info(record_id)
        if path is None or info is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        artifact = find_splat_artifact(path)
        if artifact is None:
            self.send_json({"error": "record has no .ply, .splat, or .spz artifact"}, status=HTTPStatus.NOT_FOUND)
            return
        relative = "" if path.is_file() else str(artifact.relative_to(path))
        artifact_url = f"/api/records/{record_id}/artifact"
        if relative:
            artifact_url += f"?path={relative}"
        body = splat_viewer_html(str(info["label"]), artifact_url).encode()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args: object) -> None:
        sys.stderr.write(f"service: {self.address_string()} {fmt % args}\n")

    def start_flight_session(self, flight_id: str) -> None:
        if not self.server.store.flight_exists(flight_id):
            self.send_json({"error": "flight not found"}, status=HTTPStatus.NOT_FOUND)
            return

        payload = self.read_json()
        source_name = str(payload.get("source") or "live")
        max_frames = optional_int(payload, "maxFrames")
        try:
            frame_source = make_frame_source(source_name, payload)
            session = FlightSession(
                flight_id=flight_id,
                source_name=source_name,
                frame_source=frame_source,
                work_root=self.server.session_work_root,
                max_frames=max_frames,
                intrinsics=load_intrinsics("forward"),
                enable_pose_estimation=optional_bool(payload, "estimatePose") is not False,
            )
            status = self.server.sessions.start(session)
        except (OSError, RuntimeError, ValueError) as exc:
            self.send_json({"error": str(exc)}, status=HTTPStatus.CONFLICT)
            return

        self.server.store.update_flight(
            flight_id,
            mode="manual",
            metadata={"sessionStatus": "recording", "sessionId": status.session_id, "sessionSource": source_name},
        )
        self.send_json(status.as_dict())

    def stop_flight_session(self, flight_id: str) -> None:
        status = self.server.sessions.stop(flight_id)
        if status is None:
            self.send_json({"error": "no active session"}, status=HTTPStatus.NOT_FOUND)
            return

        record_id = status.record_id
        session_obj = self.server.sessions.get(flight_id)
        if status.frames > 0 and record_id is None:
            record_id = self.server.store.import_record(
                flight_id,
                "frames",
                f"Session frames {status.session_id}",
                "image/jpeg-sequence",
                Path(status.frame_dir),
            )
            if session_obj is not None:
                session_obj.set_record_id(record_id)

        pose_record_id = status.pose_record_id
        if (
            pose_record_id is None
            and status.pose_path
            and Path(status.pose_path).is_file()
            and Path(status.pose_path).stat().st_size > 0
        ):
            pose_record_id = self.server.store.import_record(
                flight_id,
                "pose-track",
                f"Session pose track {status.session_id}",
                "application/jsonl",
                Path(status.pose_path),
            )
            if session_obj is not None:
                session_obj.set_pose_record_id(pose_record_id)

        metrics = {
            "frames": status.frames,
            "bytes": status.bytes,
            "duration": format_duration(status.duration_seconds),
        }
        self.server.store.update_flight(
            flight_id,
            duration=format_duration(status.duration_seconds),
            metadata={
                "sessionStatus": "stopped" if not status.error else "error",
                "sessionId": status.session_id,
                "sessionSource": status.source,
                "sessionError": status.error or "",
            },
            metrics=metrics,
        )
        refreshed = self.server.sessions.status(flight_id)
        self.send_json((refreshed or status).as_dict() | {"recordId": record_id})

    def connect_wifi(self) -> None:
        payload = self.read_json()
        iface = str(payload.get("iface") or "")
        ssid = str(payload.get("ssid") or "")
        if not iface or not ssid:
            self.send_json({"error": "iface and ssid are required"}, status=HTTPStatus.BAD_REQUEST)
            return
        if payload.get("confirmDisconnect") is not True:
            self.send_json({"error": "confirmDisconnect must be true"}, status=HTTPStatus.BAD_REQUEST)
            return

        previous = platform_current_wifi_connection(iface)
        result = platform_connect_wifi(iface, ssid, optional_str(payload, "password"))
        if result["ok"]:
            self.server.wifi_previous[iface] = previous or ""
        self.send_json({"iface": iface, "ssid": ssid, "previousConnection": previous, **result})

    def reconnect_wifi(self) -> None:
        payload = self.read_json()
        iface = str(payload.get("iface") or "")
        if not iface:
            self.send_json({"error": "iface is required"}, status=HTTPStatus.BAD_REQUEST)
            return
        target = str(payload.get("ssid") or self.server.wifi_previous.get(iface) or os.environ.get("HOME_SSID", ""))
        if not target:
            self.send_json({"error": "ssid is required; no previous connection is known"}, status=HTTPStatus.BAD_REQUEST)
            return

        result = platform_reconnect_wifi(iface, target, optional_str(payload, "password"))
        self.send_json({"iface": iface, "ssid": target, **result})

    def discover_drones(self) -> None:
        payload = self.read_json()
        iface = str(payload.get("iface") or default_wifi_interface())
        rescan = payload.get("rescan") is not False
        try:
            access_points = scan_access_points(iface, rescan=rescan)
        except (OSError, subprocess.CalledProcessError) as exc:
            self.send_json({"error": str(exc), "discovered": []}, status=HTTPStatus.SERVICE_UNAVAILABLE)
            return
        discovered = []
        for ap in access_points:
            if not ap.likely_drone:
                continue
            drone_id = self.server.store.upsert_discovered_drone(
                ssid=ap.ssid,
                bssid=ap.bssid or None,
                iface=iface,
                signal=ap.signal,
            )
            discovered.append({"id": drone_id, **asdict(ap)})
        self.send_json({"iface": iface, "discovered": discovered, "state": self.server.store.state()})

    def reveal_record(self, record_id: str) -> None:
        path = self.server.store.record_path(record_id)
        if path is None:
            self.send_json({"error": "record not found or missing blob"}, status=HTTPStatus.NOT_FOUND)
            return
        result = reveal_path(path)
        self.send_json({"path": str(path), **result})

    def compute_pose_track(self, flight_id: str) -> None:
        if not self.server.store.flight_exists(flight_id):
            self.send_json({"error": "flight not found"}, status=HTTPStatus.NOT_FOUND)
            return
        payload = self.read_json()
        record_id = optional_str(payload, "recordId")
        try:
            result = self.server.compute_pose_track_record(
                flight_id,
                record_id=record_id,
                fps=float(payload.get("fps") or 12),
            )
        except RuntimeError as exc:
            self.send_json({"error": str(exc)}, status=HTTPStatus.SERVICE_UNAVAILABLE)
            return
        except FileNotFoundError as exc:
            self.send_json({"error": str(exc)}, status=HTTPStatus.NOT_FOUND)
            return
        self.send_json(result)

    def set_drone_guidance(self, drone_id: str) -> None:
        """Set guidance for one drone. Body may include any of: target {x,y,z} or
        null to clear, trajectory [[x,y,z],...] (+ loop), style [...], policyId."""

        payload = self.read_json()
        runtime = self.server.runtime
        if "target" in payload:
            target = payload.get("target")
            if target is None:
                runtime.set_target(drone_id, None)
            else:
                runtime.set_target(drone_id, (float(target[0]), float(target[1]), float(target[2])))
        if "trajectory" in payload:
            waypoints = [(float(w[0]), float(w[1]), float(w[2])) for w in payload.get("trajectory") or []]
            runtime.set_trajectory(drone_id, waypoints, loop=bool(payload.get("loop", False)))
        if "style" in payload:
            runtime.set_style(drone_id, [float(v) for v in payload.get("style") or []])
        if "policyId" in payload:
            runtime.select_policy(drone_id, payload.get("policyId"))
        self.send_json({"guidance": runtime.guidance_status()})

    def start_drone_camera(self, drone_id: str) -> None:
        """Attach a live camera source for a drone so frames flow to the VLA hub
        and the world model.

        Body either ``{"framesDir": "/path"}`` (DirectoryFrameSource replay, also
        useful without hardware) or live camera config
        ``{"iface": "...", "droneIp": "...", ...}``.
        """

        payload = self.read_json()
        frames_dir = optional_str(payload, "framesDir")
        try:
            if frames_dir:
                source = DirectoryFrameSource(frames_dir, fps=float(payload.get("fps") or 10.0))
            else:
                iface = optional_str(payload, "iface")
                if not iface:
                    self.send_json({"error": "iface or framesDir required"}, status=HTTPStatus.BAD_REQUEST)
                    return
                config = LiveDroneFrameSourceConfig(
                    iface=iface,
                    local_ip=str(payload.get("localIp") or ""),
                    drone_ip=str(payload.get("droneIp") or "192.168.1.1"),
                    use_rtsp=bool(payload.get("useRtsp", True)),
                )
                source = LiveDroneFrameSource(config)
            self.server.runtime.attach_frame_source(drone_id, source)
        except (FileNotFoundError, ValueError, OSError) as exc:
            self.send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return
        self.send_json({"ingestion": self.server.runtime.ingestion_status()})

    def bootstrap_world_model(self, flight_drones: Any) -> None:
        """COLMAP-union cross-drone bootstrap from recorded flights.

        ``flight_drones`` may be a list of flight ids (each treated as its own
        drone, labelled by flight id) or a mapping ``{flightId: droneId}`` so the
        resulting transforms apply to the matching live runtime drone.
        """

        if isinstance(flight_drones, dict):
            mapping = {str(k): str(v) for k, v in flight_drones.items()}
        elif isinstance(flight_drones, list):
            mapping = {str(f): str(f) for f in flight_drones}
        else:
            self.send_json({"error": "flightIds must be a list or {flightId: droneId} object"}, status=HTTPStatus.BAD_REQUEST)
            return

        drone_frames: dict[str, list[str]] = {}
        for flight_id, drone_id in mapping.items():
            if not self.server.store.flight_exists(flight_id):
                self.send_json({"error": f"flight not found: {flight_id}"}, status=HTTPStatus.NOT_FOUND)
                return
            frame_record = self.server._latest_frame_record(flight_id)
            if frame_record is None:
                self.send_json({"error": f"no frames record for flight: {flight_id}"}, status=HTTPStatus.NOT_FOUND)
                return
            frame_dir = self.server.store.record_path(str(frame_record["id"]))
            if frame_dir is None or not frame_dir.is_dir():
                self.send_json({"error": f"frame blob missing for flight: {flight_id}"}, status=HTTPStatus.NOT_FOUND)
                return
            frames = [str(p) for p in sorted(frame_dir.glob("*.jpg"))]
            if not frames:
                self.send_json({"error": f"no .jpg frames for flight: {flight_id}"}, status=HTTPStatus.NOT_FOUND)
                return
            drone_frames.setdefault(drone_id, []).extend(frames)

        work_dir = self.server.reconstruction_root / "world_bootstrap"
        try:
            result = self.server.runtime.bootstrap_world_model(drone_frames, work_dir)
        except Exception as exc:
            self.send_json({"error": str(exc)}, status=HTTPStatus.CONFLICT)
            return
        self.send_json(result)

    def start_reconstruction(self, flight_id: str) -> None:
        if not self.server.store.flight_exists(flight_id):
            self.send_json({"error": "flight not found"}, status=HTTPStatus.NOT_FOUND)
            return
        payload = self.read_json()
        frame_record = self.server._latest_frame_record(flight_id, record_id=optional_str(payload, "recordId"))
        if frame_record is None:
            self.send_json({"error": "no frames record available"}, status=HTTPStatus.NOT_FOUND)
            return
        pose_record = self.server._latest_pose_record(flight_id)
        try:
            job = self.server.reconstructions.start(
                flight_id=flight_id,
                frame_record=frame_record,
                pose_record=pose_record,
                max_images=optional_int(payload, "maxImages"),
                max_iterations=optional_int(payload, "maxIterations"),
                fps=float(payload.get("fps") or 12),
            )
        except RuntimeError as exc:
            self.send_json({"error": str(exc)}, status=HTTPStatus.CONFLICT)
            return
        self.send_json(self.server.reconstruction_status(flight_id) | {"jobId": job.id})

    def stop_reconstruction(self, flight_id: str) -> None:
        status = self.server.reconstructions.stop(flight_id)
        if status is None:
            self.send_json({"error": "no reconstruction job"}, status=HTTPStatus.NOT_FOUND)
            return
        self.send_json(self.server.reconstruction_status(flight_id))

    def export_record(self, record_id: str) -> None:
        payload = self.read_json()
        fmt = str(payload.get("format") or "mjpeg").lower()
        info = self.server.store.record_info(record_id)
        path = self.server.store.record_path(record_id)
        if info is None or path is None or not path.is_dir():
            self.send_json({"error": "frame record not found"}, status=HTTPStatus.NOT_FOUND)
            return
        if fmt not in {"mjpeg", "mp4"}:
            self.send_json({"error": "format must be mjpeg or mp4"}, status=HTTPStatus.BAD_REQUEST)
            return
        fps = max(1.0, min(60.0, float(payload.get("fps") or 12)))
        try:
            exported = export_frame_dir(path, self.server.export_root, fmt=fmt, fps=fps)
            label = f"{info['label']} {fmt.upper()} export"
            mime = "video/mp4" if fmt == "mp4" else "multipart/x-mixed-replace"
            new_id = self.server.store.import_record(str(info["flightId"]), fmt, label, mime, exported)
        except RuntimeError as exc:
            self.send_json({"error": str(exc)}, status=HTTPStatus.CONFLICT)
            return
        self.send_json({"id": new_id, "path": str(exported), "format": fmt})


class ControlStationServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], store: ControlStationStore) -> None:
        super().__init__(server_address, ControlStationHandler)
        self.store = store
        self.manual = ManualControlSession(ManualControlConfig())
        self.manual_lock = threading.RLock()
        self.manual_transport = ManualDroneTransport.from_env()
        self.session_work_root = store.db_path.parent / "session_work"
        self.export_root = store.db_path.parent / "exports"
        self.reconstruction_root = store.db_path.parent / "reconstructions"
        self.export_root.mkdir(parents=True, exist_ok=True)
        self.sessions = FlightSessionManager(self.session_work_root)
        self.reconstructions = ReconstructionManager(store=store, work_root=self.reconstruction_root)
        self.runtime = RuntimeManager(
            config=RuntimeManagerConfig(
                control_hz=float(os.environ.get("DRONE_RUNTIME_HZ", "20")),
                dry_run=env_bool("DRONE_RUNTIME_DRY_RUN", True),
                enable_io=env_bool("DRONE_RUNTIME_ENABLE_IO", False),
                local_vla_command=env_command("DRONE_LOCAL_VLA_COMMAND"),
                local_vla_timeout_seconds=float(os.environ.get("DRONE_LOCAL_VLA_TIMEOUT", "0.25")),
                batched_vla_command=env_command("DRONE_BATCHED_VLA_COMMAND"),
                batched_vla_timeout_seconds=float(os.environ.get("DRONE_BATCHED_VLA_TIMEOUT", "0.25")),
                batch_max_wait_seconds=float(os.environ.get("DRONE_BATCH_MAX_WAIT", "0.025")),
                vla_log_path=os.environ.get("DRONE_VLA_LOG_PATH") or None,
                policy_commands=_parse_policy_commands(os.environ.get("DRONE_POLICY_COMMANDS")),
            )
        )
        self.runtime.configure_drones(load_runtime_configs())
        self.sim = SimSession()
        self.coordinator = CoordinatorScheduler(tick_hz=float(os.environ.get("DRONE_COORDINATOR_HZ", "1")))
        self.vlm = make_vlm_coordinator()
        self.wifi_previous: dict[str, str] = {}
        self.manual_loop_running = True
        self.manual_thread = threading.Thread(target=self._manual_loop, name="manual-control-loop", daemon=True)
        self.manual_thread.start()
        self.autonomy_loop_running = True
        self.autonomy_thread = threading.Thread(target=self._autonomy_loop, name="autonomy-loop", daemon=True)
        self.autonomy_thread.start()

    def manual_status(self, action: object | None = None) -> dict[str, object]:
        payload = {
            "state": self.manual.state.value,
            "armed": self.manual.armed,
            "faultReason": self.manual.fault_reason,
            "stopReason": self.manual.stop_reason,
            "current": self.manual.current_action_dict(),
            "transport": self.manual_transport.status().as_dict(),
        }
        if action is not None:
            payload["action"] = action_to_dict(action)
        return payload

    def config_status(self) -> dict[str, object]:
        return {
            "platform": platform.system() or "Unknown",
            "network": platform_network_summary(),
            "manual": self.manual_transport.config_dict(),
            "policy": {
                "maxThrottle": self.manual.config.max_throttle,
                "commandHz": self.manual.config.command_hz,
                "throttleSlewPerSecond": self.manual.config.throttle_slew_per_second,
                "heartbeatTimeoutSeconds": self.manual.config.heartbeat_timeout_seconds,
            },
            "camera": {
                "iface": os.environ.get("DRONE_IFACE", default_wifi_interface()),
                "localIp": os.environ.get("DRONE_CAMERA_LOCAL_IP", ""),
                "droneIp": os.environ.get("DRONE_IP", "192.168.1.1"),
                "rtspPort": int(os.environ.get("DRONE_RTSP_PORT", "7070")),
                "videoPort": int(os.environ.get("DRONE_CAMERA_VIDEO_PORT", "32124")),
                "auxPort": int(os.environ.get("DRONE_CAMERA_AUX_PORT", "32125")),
                "bindDevice": env_bool("DRONE_CAMERA_BIND_DEVICE", False),
                "useRtsp": env_bool("DRONE_CAMERA_USE_RTSP", True),
            },
            "linkCapabilities": {
                "mixedLinks": True,
                "directUdp": True,
                "espSerial": True,
                "radioModel": "one independent radio association per drone AP",
            },
            "reconstruction": self.reconstructions.tools_status(),
            "runtime": {
                "dryRun": self.runtime.config.dry_run,
                "enableIo": self.runtime.config.enable_io,
                "controlHz": self.runtime.config.control_hz,
                "localVlaConfigured": bool(self.runtime.config.local_vla_command),
                "internetVlmConfigured": self.vlm.available,
            },
        }

    def runtime_status(self) -> dict[str, object]:
        status = self.runtime.snapshots()
        status["mission"] = self.mission_progress()
        return status

    def mission_progress(self) -> dict[str, object]:
        return self._advance_mission().as_dict()

    def _advance_mission(self) -> MissionProgress:
        snapshots = self.runtime.snapshot_objects()
        if self.coordinator.mission is not None and self.vlm.available:
            progress = self.vlm.step(self.coordinator.mission, [snapshot.as_dict() for snapshot in snapshots])
            if progress.state == "faulted":
                fallback = self.coordinator.step(snapshots)
                progress = type(progress)(
                    mission_id=progress.mission_id,
                    state=fallback.state,
                    assignments=fallback.assignments,
                    notes=[*progress.notes, "vlm_failed_using_scheduler", *fallback.notes],
                )
        else:
            progress = self.coordinator.step(snapshots)
        self.runtime.apply_mission_progress(progress)
        if progress.tool_calls:
            # Low-frequency VLM guidance -> guidance bus -> hi-frequency conditioning.
            self.runtime.apply_guidance_tool_calls(progress.tool_calls)
        if self.coordinator.mission is not None:
            self.runtime.heartbeat_all()
        return progress

    def send_manual_action(self, action: object | None) -> bool:
        sent = self.manual_transport.send(action)
        if sent:
            self.manual.ack()
        return sent

    def compute_pose_track_record(
        self,
        flight_id: str,
        *,
        record_id: str | None = None,
        fps: float = 12.0,
    ) -> dict[str, object]:
        frames_record = self._latest_frame_record(flight_id, record_id=record_id)
        if frames_record is None:
            raise FileNotFoundError("no frames record available")
        frame_dir = self.store.record_path(str(frames_record["id"]))
        if frame_dir is None or not frame_dir.is_dir():
            raise FileNotFoundError("frames record blob missing")

        out_path = self.session_work_root / f"replay-{frames_record['id']}" / "pose.jsonl"
        out_path.parent.mkdir(parents=True, exist_ok=True)
        status = replay_directory(frame_dir, out_path, fps=fps)
        pose_record_id = self.store.import_record(
            flight_id,
            "pose-track",
            f"Replay pose track {frames_record['id']}",
            "application/jsonl",
            out_path,
        )
        return {
            "id": pose_record_id,
            "status": status.as_dict(),
            "sourceRecordId": frames_record["id"],
        }

    def pose_status(self, flight_id: str) -> dict[str, object]:
        session = self.sessions.get(flight_id)
        if session is not None and session.estimator is not None:
            return {
                "flightId": flight_id,
                "live": True,
                "status": session.estimator.status().as_dict(),
            }
        record = self._latest_pose_record(flight_id)
        if record is None:
            return {"flightId": flight_id, "live": False, "status": self._missing_pose_status(flight_id)}
        return {
            "flightId": flight_id,
            "live": False,
            "status": {"state": "stored", "recordId": record["id"]},
        }

    def pose_track(self, flight_id: str, *, since: int = -1) -> dict[str, object]:
        session = self.sessions.get(flight_id)
        if session is not None and session.estimator is not None:
            poses = session.estimator_poses(since)
            return {
                "flightId": flight_id,
                "live": True,
                "poses": poses,
                "status": session.estimator.status().as_dict(),
            }
        record = self._latest_pose_record(flight_id)
        if record is None:
            return {"flightId": flight_id, "live": False, "poses": [], "status": self._missing_pose_status(flight_id)}
        path = self.store.record_path(record["id"])
        poses = load_pose_track(path) if path else []
        if since >= 0:
            poses = [p for p in poses if int(p.get("frameIndex", -1)) > since]
        return {
            "flightId": flight_id,
            "live": False,
            "poses": poses,
            "status": {"state": "stored", "framesProcessed": len(poses)},
        }

    def reconstruction_status(self, flight_id: str) -> dict[str, object]:
        status = self.reconstructions.status(flight_id)
        latest_splat = self._latest_splat_record(flight_id)
        return {
            "flightId": flight_id,
            "job": status,
            "tools": self.reconstructions.tools_status(),
            "latestSplatRecord": latest_splat,
        }

    def _missing_pose_status(self, flight_id: str) -> dict[str, object]:
        frame_record = self._latest_frame_record(flight_id)
        if frame_record is None:
            return {"state": "no_estimator", "estimatorAvailable": estimator_available()}
        if not estimator_available():
            return {
                "state": "no_estimator",
                "estimatorAvailable": False,
                "framesAvailable": True,
                "sourceRecordId": frame_record["id"],
                "lastError": "opencv-python is required for pose estimation",
            }
        return {
            "state": "not_computed",
            "estimatorAvailable": True,
            "framesAvailable": True,
            "sourceRecordId": frame_record["id"],
        }

    def _latest_frame_record(self, flight_id: str, *, record_id: str | None = None) -> dict[str, object] | None:
        state = self.store.state()
        for drone in state.get("drones", []):
            for flight in drone.get("flights", []):
                if flight.get("id") != flight_id:
                    continue
                frame_records = [r for r in flight.get("records", []) if r.get("type") == "frames"]
                if record_id:
                    return next((r for r in frame_records if r.get("id") == record_id), None)
                if frame_records:
                    return frame_records[-1]
        return None

    def _latest_pose_record(self, flight_id: str) -> dict[str, object] | None:
        state = self.store.state()
        for drone in state.get("drones", []):
            for flight in drone.get("flights", []):
                if flight.get("id") != flight_id:
                    continue
                pose_records = [r for r in flight.get("records", []) if r.get("type") == "pose-track"]
                if not pose_records:
                    return None
                return pose_records[-1]
        return None

    def _latest_splat_record(self, flight_id: str) -> dict[str, object] | None:
        state = self.store.state()
        for drone in state.get("drones", []):
            for flight in drone.get("flights", []):
                if flight.get("id") != flight_id:
                    continue
                records = [r for r in flight.get("records", []) if r.get("type") == "gaussian-splat"]
                if not records:
                    return None
                return records[-1]
        return None

    def server_close(self) -> None:
        self.manual_loop_running = False
        self.manual_thread.join(timeout=1.0)
        self.autonomy_loop_running = False
        self.autonomy_thread.join(timeout=1.0)
        self.sim.stop()
        self.runtime.stop_all()
        self.runtime.stop_world_model()
        self.runtime.close_vla()
        self.reconstructions.stop_all()
        self.sessions.stop_all()
        self.manual_transport.close()
        super().server_close()

    def _manual_loop(self) -> None:
        interval = self.manual.config.command_interval_seconds
        while self.manual_loop_running:
            with self.manual_lock:
                action = self.manual.tick()
                self.send_manual_action(action)
            time.sleep(interval)

    def _autonomy_loop(self) -> None:
        interval = 1.0 / max(0.1, float(os.environ.get("DRONE_COORDINATOR_HZ", "1")))
        while self.autonomy_loop_running:
            try:
                if self.coordinator.mission is not None:
                    self._advance_mission()
            except Exception as exc:
                sys.stderr.write(f"service: autonomy loop error: {exc}\n")
            time.sleep(interval)


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
    if platform.system().lower() != "linux":
        summary = platform_network_summary()
        return {
            "available": bool(summary["interfaces"]),
            "platform": summary["platform"],
            "interfaces": summary["interfaces"],
            "simultaneousManagedLikely": False,
            "recommendation": summary["notes"],
        }
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


def current_wifi_connection(iface: str) -> str:
    try:
        output = subprocess.check_output(
            ["nmcli", "-t", "-f", "DEVICE,CONNECTION", "dev", "status"],
            text=True,
            stderr=subprocess.STDOUT,
        )
    except (OSError, subprocess.CalledProcessError):
        return ""
    for line in output.splitlines():
        device, _, connection = line.partition(":")
        if device == iface:
            return connection
    return ""


def run_nmcli(args: list[str]) -> dict[str, object]:
    try:
        completed = subprocess.run(
            ["nmcli", *args],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"ok": False, "returnCode": -1, "output": str(exc)}
    return {
        "ok": completed.returncode == 0,
        "returnCode": completed.returncode,
        "output": completed.stdout.strip(),
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


def make_frame_source(source_name: str, payload: dict[str, object]) -> object:
    if source_name == "directory":
        frame_dir = resolve_repo_path(str(payload.get("frameDir") or ""))
        if frame_dir is None or not frame_dir.is_dir():
            raise ValueError("directory source requires frameDir inside the repository")
        fps = float(payload.get("fps") or 12)
        return DirectoryFrameSource(frame_dir, fps=max(1.0, min(60.0, fps)))
    if source_name != "live":
        raise ValueError("source must be live or directory")

    return LiveDroneFrameSource(
        LiveDroneFrameSourceConfig(
            iface=str(payload.get("iface") or os.environ.get("DRONE_IFACE", default_wifi_interface())),
            local_ip=str(payload.get("localIp") or os.environ.get("DRONE_CAMERA_LOCAL_IP", "")),
            drone_ip=str(payload.get("droneIp") or os.environ.get("DRONE_IP", "192.168.1.1")),
            rtsp_port=int(payload.get("rtspPort") or os.environ.get("DRONE_RTSP_PORT", "7070")),
            video_port=int(payload.get("videoPort") or os.environ.get("DRONE_CAMERA_VIDEO_PORT", "32124")),
            aux_port=int(payload.get("auxPort") or os.environ.get("DRONE_CAMERA_AUX_PORT", "32125")),
            drone_video_port=int(payload.get("droneVideoPort") or os.environ.get("DRONE_CAMERA_DRONE_VIDEO_PORT", "53797")),
            bind_device=env_bool("DRONE_CAMERA_BIND_DEVICE", False),
            use_rtsp=env_bool("DRONE_CAMERA_USE_RTSP", True),
        )
    )


def load_runtime_configs() -> list[object]:
    configured = os.environ.get("DRONE_RUNTIME_CONFIG", "")
    candidates = []
    if configured:
        candidates.append(Path(configured))
    candidates.extend([REPO_ROOT / "config" / "drones.local.json", REPO_ROOT / "config" / "drones.example.json"])
    for path in candidates:
        if path.is_file():
            return load_config(path)
    return []


def make_vlm_coordinator() -> VLMCoordinator:
    endpoint = os.environ.get("DRONE_VLM_ENDPOINT", "")
    if not endpoint:
        return VLMCoordinator()
    client = HttpVLMClient(
        HttpVLMConfig(
            endpoint=endpoint,
            api_key=os.environ.get("DRONE_VLM_API_KEY", ""),
            timeout_seconds=float(os.environ.get("DRONE_VLM_TIMEOUT", "5")),
        )
    )
    return VLMCoordinator(model_step=client.step)


def env_command(name: str) -> list[str] | None:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return None
    try:
        value = json.loads(raw)
    except json.JSONDecodeError:
        return shlex.split(raw)
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"{name} must be a JSON string list or shell-like command string")
    return value


def env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _parse_policy_commands(raw: str | None) -> dict[str, list[str]] | None:
    """Parse DRONE_POLICY_COMMANDS: a JSON object of policyId -> command (string
    or string list) used for select_policy per-policy model processes."""

    if not raw or not raw.strip():
        return None
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError("DRONE_POLICY_COMMANDS must be a JSON object")
    commands: dict[str, list[str]] = {}
    for policy_id, command in value.items():
        if isinstance(command, str):
            commands[str(policy_id)] = shlex.split(command)
        elif isinstance(command, list) and all(isinstance(item, str) for item in command):
            commands[str(policy_id)] = command
        else:
            raise ValueError("each policy command must be a string or string list")
    return commands


def optional_str(payload: dict[str, object], key: str) -> str | None:
    if key not in payload:
        return None
    return str(payload[key])


def optional_int(payload: dict[str, object], key: str) -> int | None:
    if key not in payload or payload[key] in {None, ""}:
        return None
    return int(payload[key])


def optional_float(payload: dict[str, object], key: str) -> float | None:
    if key not in payload or payload[key] in {None, ""}:
        return None
    return float(payload[key])


def optional_bool(payload: dict[str, object], key: str) -> bool | None:
    if key not in payload or payload[key] in {None, ""}:
        return None
    value = payload[key]
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


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


def reveal_path(path: Path) -> dict[str, object]:
    target = path if path.is_file() else path
    system = platform.system().lower()
    if system == "darwin":
        args = ["open", "-R", str(target)] if target.is_file() else ["open", str(target)]
    elif system == "windows":
        args = ["explorer", f"/select,{target}"] if target.is_file() else ["explorer", str(target)]
    else:
        args = ["xdg-open", str(target.parent if target.is_file() else target)]
    try:
        completed = subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"ok": False, "returnCode": -1, "output": str(exc)}
    return {"ok": completed.returncode == 0, "returnCode": completed.returncode, "output": completed.stdout.strip()}


def export_frame_dir(frame_dir: Path, export_root: Path, *, fmt: str, fps: float) -> Path:
    frames = sorted(frame_dir.glob("*.jpg"))
    if not frames:
        raise RuntimeError("frame record has no .jpg frames")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    export_root.mkdir(parents=True, exist_ok=True)
    if fmt == "mjpeg":
        out = export_root / f"{frame_dir.name}_{stamp}.mjpeg"
        boundary = b"--frame\r\n"
        with out.open("wb") as handle:
            for frame in frames:
                data = frame.read_bytes()
                handle.write(boundary)
                handle.write(b"Content-Type: image/jpeg\r\n")
                handle.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                handle.write(data)
                handle.write(b"\r\n")
            handle.write(b"--frame--\r\n")
        return out
    if fmt == "mp4":
        ffmpeg = shutil.which("ffmpeg")
        if not ffmpeg:
            raise RuntimeError("ffmpeg is required for MP4 export; use MJPEG export or install ffmpeg")
        out = export_root / f"{frame_dir.name}_{stamp}.mp4"
        pattern = str(frame_dir / "frame_%06d.jpg")
        alt_pattern = str(frame_dir / "frame_%05d_*.jpg")
        input_pattern = pattern if (frame_dir / "frame_000000.jpg").exists() else alt_pattern
        completed = subprocess.run(
            [
                ffmpeg,
                "-y",
                "-framerate",
                str(fps),
                "-pattern_type",
                "glob" if "*" in input_pattern else "sequence",
                "-i",
                input_pattern,
                "-pix_fmt",
                "yuv420p",
                str(out),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=120,
            check=False,
        )
        if completed.returncode != 0:
            raise RuntimeError(completed.stdout.strip() or "ffmpeg export failed")
        return out
    raise RuntimeError(f"unsupported export format: {fmt}")


def splat_viewer_html(title: str, artifact_url: str) -> str:
    escaped_title = html.escape(title)
    title_json = json.dumps(title)
    url_json = json.dumps(artifact_url)
    return f"""<!doctype html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>{escaped_title}</title>
    <style>
      html, body {{ margin: 0; width: 100%; height: 100%; overflow: hidden; background: #07090a; color: #edf1f3; font-family: ui-monospace, Menlo, Consolas, monospace; }}
      #status {{ position: fixed; left: 12px; bottom: 12px; z-index: 2; padding: 6px 10px; background: rgba(0,0,0,.72); border: 1px solid rgba(255,255,255,.12); font-size: 11px; letter-spacing: .06em; text-transform: uppercase; }}
      canvas {{ display: block; width: 100vw; height: 100vh; }}
    </style>
  </head>
  <body>
    <div id="status">LOADING</div>
    <script type="module">
      import * as SPLAT from "https://cdn.jsdelivr.net/npm/gsplat@latest/+esm";
      const label = {title_json};
      const artifactUrl = new URL({url_json}, window.location.href).toString();
      const status = document.getElementById("status");
      const scene = new SPLAT.Scene();
      const camera = new SPLAT.Camera();
      const renderer = new SPLAT.WebGLRenderer();
      const controls = new SPLAT.OrbitControls(camera, renderer.canvas);
      document.body.appendChild(renderer.canvas);
      try {{
        await SPLAT.Loader.LoadAsync(artifactUrl, scene, (progress) => {{
          const pct = Number.isFinite(progress) ? Math.round(progress * 100) : 0;
          status.textContent = pct > 0 ? `${{label}} ${{pct}}%` : `LOADING ${{label}}`;
        }});
        status.textContent = label;
        const frame = () => {{
          controls.update();
          renderer.render(scene, camera);
          requestAnimationFrame(frame);
        }};
        requestAnimationFrame(frame);
      }} catch (error) {{
        console.error(error);
        status.textContent = `LOAD FAILED: ${{error?.message || error}}`;
      }}
    </script>
  </body>
</html>
"""


def format_duration(seconds: float) -> str:
    total = max(0, int(round(seconds)))
    hours, remainder = divmod(total, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


if __name__ == "__main__":
    raise SystemExit(main())
