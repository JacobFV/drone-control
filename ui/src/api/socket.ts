import { useEffect, useRef, useState } from "react";
import { api, getWsUrl } from "./client";
import type { WsSnapshot } from "./types";

type Command = Record<string, unknown>;

/**
 * Live snapshot stream. Prefers the WebSocket push channel announced by the
 * service; if it is unavailable (e.g. the ``websockets`` package is missing) it
 * transparently falls back to HTTP polling. Returns the latest snapshot plus a
 * ``send`` for realtime commands and a transport label.
 */
export function useLiveSnapshot(): {
  snapshot: WsSnapshot | null;
  transport: "ws" | "poll" | "connecting";
  send: (command: Command) => void;
} {
  const [snapshot, setSnapshot] = useState<WsSnapshot | null>(null);
  const [transport, setTransport] = useState<"ws" | "poll" | "connecting">("connecting");
  const socketRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    let cancelled = false;
    let pollTimer = 0;
    let reconnectTimer = 0;

    const poll = async () => {
      const [session, runtime, manual] = await Promise.all([
        api.getSessionStatus(),
        api.getRuntimeStatus(),
        api.getManualStatus(),
      ]);
      if (cancelled) return;
      setSnapshot({
        session: session ?? { active: false },
        runtime: (runtime as WsSnapshot["runtime"]) ?? {},
        manual: manual ?? { state: "unknown" },
      });
    };

    const startPolling = () => {
      setTransport("poll");
      void poll();
      pollTimer = window.setInterval(poll, 700);
    };

    const connect = async () => {
      const url = await getWsUrl();
      if (cancelled) return;
      if (!url) {
        startPolling();
        return;
      }
      try {
        const socket = new WebSocket(url);
        socketRef.current = socket;
        socket.onopen = () => !cancelled && setTransport("ws");
        socket.onmessage = (event) => {
          if (cancelled) return;
          try {
            const message = JSON.parse(event.data);
            if (message.type === "status") setSnapshot(message.data as WsSnapshot);
          } catch {
            /* ignore malformed frame */
          }
        };
        socket.onclose = () => {
          if (cancelled) return;
          socketRef.current = null;
          // Reconnect once; if it keeps failing, fall back to polling.
          reconnectTimer = window.setTimeout(startPolling, 1500);
        };
        socket.onerror = () => socket.close();
      } catch {
        startPolling();
      }
    };

    void connect();
    return () => {
      cancelled = true;
      window.clearInterval(pollTimer);
      window.clearTimeout(reconnectTimer);
      socketRef.current?.close();
    };
  }, []);

  const send = (command: Command) => {
    const socket = socketRef.current;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify(command));
    }
  };

  return { snapshot, transport, send };
}
