from __future__ import annotations

import json
import subprocess
import threading
from dataclasses import dataclass
from typing import Any


class LocalVLAError(RuntimeError):
    pass


@dataclass(slots=True)
class LocalVLAConfig:
    command: list[str]
    timeout_seconds: float = 0.25


class LocalVLAClient:
    """
    JSON-lines adapter for a local VLA process.

    The process receives one JSON object per stdin line and must return one JSON
    object per stdout line. The returned object is still validated by
    controllers.vla before it can become a DroneAction.
    """

    def __init__(self, config: LocalVLAConfig) -> None:
        if not config.command:
            raise ValueError("local VLA command is required")
        self.config = config
        self._lock = threading.Lock()
        self._process: subprocess.Popen[str] | None = None

    def step(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            process = self._ensure_process()
            assert process.stdin is not None
            assert process.stdout is not None
            try:
                process.stdin.write(json.dumps(payload, separators=(",", ":")) + "\n")
                process.stdin.flush()
                line = _readline_with_timeout(process.stdout, self.config.timeout_seconds)
            except (BrokenPipeError, OSError) as exc:
                self.close()
                raise LocalVLAError(f"local VLA process IO failed: {exc}") from exc
            if not line:
                self.close()
                raise LocalVLAError("local VLA process timed out")
            try:
                result = json.loads(line)
            except json.JSONDecodeError as exc:
                raise LocalVLAError(f"local VLA returned invalid JSON: {exc}") from exc
            if not isinstance(result, dict):
                raise LocalVLAError("local VLA output must be an object")
            return result

    def close(self) -> None:
        process = self._process
        self._process = None
        if process is None:
            return
        for stream in (process.stdin, process.stdout, process.stderr):
            try:
                if stream is not None:
                    stream.close()
            except OSError:
                pass
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)

    def _ensure_process(self) -> subprocess.Popen[str]:
        if self._process is not None and self._process.poll() is None:
            return self._process
        self._process = subprocess.Popen(
            self.config.command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        return self._process


class BatchLocalVLAClient:
    """
    JSON-lines adapter for a local *batched* VLA process.

    Per control window the hub hands this client a list of per-drone payloads.
    The client writes a single line ``{"batch": [...]}`` to the process stdin and
    reads a single line ``{"results": [...]}`` from its stdout. Each result is
    still validated by ``controllers.vla.parse_vla_output`` before it becomes a
    DroneAction, and the safety wrapper clamps it after that.

    Batching N drones into one process round-trip is the whole point: a single
    forward pass on the GPU instead of N independent calls.
    """

    def __init__(self, config: LocalVLAConfig, *, startup_timeout_seconds: float = 30.0) -> None:
        if not config.command:
            raise ValueError("local VLA command is required")
        self.config = config
        self.startup_timeout_seconds = startup_timeout_seconds
        self._lock = threading.Lock()
        self._process: subprocess.Popen[str] | None = None
        self._warmed = False

    def step_batch(self, payloads: list[dict[str, Any]]) -> list[dict[str, Any]]:
        with self._lock:
            process = self._ensure_process()
            assert process.stdin is not None
            assert process.stdout is not None
            # The first call after spawn absorbs interpreter/CUDA cold-start, so
            # grant a generous startup window before falling back to the tight
            # per-tick timeout for warm calls.
            timeout = self.config.timeout_seconds if self._warmed else self.startup_timeout_seconds
            try:
                line_in = json.dumps({"batch": payloads}, separators=(",", ":")) + "\n"
                process.stdin.write(line_in)
                process.stdin.flush()
                line = _readline_with_timeout(process.stdout, timeout)
            except (BrokenPipeError, OSError) as exc:
                self.close()
                raise LocalVLAError(f"local VLA process IO failed: {exc}") from exc
            if not line:
                self.close()
                raise LocalVLAError("local VLA process timed out")
            try:
                result = json.loads(line)
            except json.JSONDecodeError as exc:
                raise LocalVLAError(f"local VLA returned invalid JSON: {exc}") from exc
            if not isinstance(result, dict) or not isinstance(result.get("results"), list):
                raise LocalVLAError("local VLA batch output must be an object with a 'results' list")
            self._warmed = True
            return result["results"]

    def close(self) -> None:
        process = self._process
        self._process = None
        self._warmed = False
        if process is None:
            return
        for stream in (process.stdin, process.stdout, process.stderr):
            try:
                if stream is not None:
                    stream.close()
            except OSError:
                pass
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)

    def _ensure_process(self) -> subprocess.Popen[str]:
        if self._process is not None and self._process.poll() is None:
            return self._process
        self._warmed = False
        self._process = subprocess.Popen(
            self.config.command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        return self._process


def _readline_with_timeout(stream: object, timeout: float) -> str:
    result: list[str] = []
    error: list[BaseException] = []

    def target() -> None:
        try:
            result.append(stream.readline())
        except BaseException as exc:  # pragma: no cover - defensive thread boundary
            error.append(exc)

    thread = threading.Thread(target=target, daemon=True)
    thread.start()
    thread.join(timeout=max(0.001, timeout))
    if thread.is_alive():
        return ""
    if error:
        raise LocalVLAError(str(error[0]))
    return result[0] if result else ""
