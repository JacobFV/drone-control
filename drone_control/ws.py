"""
Realtime WebSocket transport for the desktop UI.

Runs an asyncio ``websockets`` server on its own thread, alongside the sync
``ThreadingHTTPServer`` in ``service.py``. It pushes a live status snapshot to
every connected client at a fixed rate and accepts a small set of command
messages (delegated to a handler). Camera video stays on MJPEG/JPEG over HTTP;
this channel carries state, trajectories, world-model, and segmentation.

Graceful degradation: if the ``websockets`` package is unavailable, ``start()``
returns ``None`` and the UI falls back to HTTP polling.
"""

from __future__ import annotations

import asyncio
import json
import threading
from typing import Any, Callable


def available() -> bool:
    try:
        import websockets  # noqa: F401
    except Exception:
        return False
    return True


class WebSocketHub:
    def __init__(
        self,
        host: str,
        status_provider: Callable[[], dict[str, Any]],
        command_handler: Callable[[dict[str, Any]], dict[str, Any]],
        *,
        hz: float = 20.0,
    ) -> None:
        self.host = host
        self.status_provider = status_provider
        self.command_handler = command_handler
        self.interval = 1.0 / max(1.0, hz)

        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._clients: set[Any] = set()
        self._server: Any | None = None
        self._port: int | None = None
        self._ready = threading.Event()

    # ------------------------------------------------------------- lifecycle

    def start(self) -> str | None:
        if not available():
            return None
        self._thread = threading.Thread(target=self._run, name="ws-hub", daemon=True)
        self._thread.start()
        self._ready.wait(timeout=5.0)
        if self._port is None:
            return None
        return f"ws://{self.host}:{self._port}"

    def stop(self) -> None:
        loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ----------------------------------------------------------------- internal

    def _run(self) -> None:
        import websockets

        async def main() -> None:
            self._loop = asyncio.get_running_loop()
            self._server = await websockets.serve(self._handler, self.host, 0)
            sock = next(iter(self._server.sockets))
            self._port = sock.getsockname()[1]
            self._ready.set()
            broadcaster = asyncio.create_task(self._broadcast_loop())
            try:
                await asyncio.Future()  # run until loop.stop()
            except asyncio.CancelledError:
                pass
            finally:
                broadcaster.cancel()

        try:
            asyncio.run(main())
        except RuntimeError:
            # loop.stop() during asyncio.run shutdown
            pass
        finally:
            self._ready.set()

    async def _handler(self, websocket: Any) -> None:
        import websockets

        self._clients.add(websocket)
        try:
            # Push an immediate snapshot on connect.
            await websocket.send(json.dumps({"type": "status", "data": self.status_provider()}))
            async for raw in websocket:
                try:
                    message = json.loads(raw)
                except (TypeError, json.JSONDecodeError):
                    continue
                try:
                    result = self.command_handler(message)
                except Exception as exc:  # pragma: no cover - defensive
                    result = {"ok": False, "error": str(exc)}
                await websocket.send(json.dumps({"type": "ack", "data": result}))
        except websockets.ConnectionClosed:
            pass
        finally:
            self._clients.discard(websocket)

    async def _broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(self.interval)
            if not self._clients:
                continue
            try:
                payload = json.dumps({"type": "status", "data": self.status_provider()})
            except Exception:
                continue
            stale = []
            for client in list(self._clients):
                try:
                    await client.send(payload)
                except Exception:
                    stale.append(client)
            for client in stale:
                self._clients.discard(client)
