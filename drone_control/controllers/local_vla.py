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
