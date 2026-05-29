import { useState } from "react";
import { useStation } from "../store/StationContext";
import { api, downloadWorldSnapshot } from "../api/client";
import { usePolling } from "../lib/usePolling";
import { Button, KeyValue, Panel, Pill } from "./primitives";
import type { WorldSplatStatus } from "../api/types";

export function WorldModelView() {
  const { selectedFlightId } = useStation();
  const [status, setStatus] = useState<WorldSplatStatus | null>(null);
  const [busy, setBusy] = useState(false);
  const [message, setMessage] = useState("");

  usePolling(async () => {
    const result = await api.getWorldSplatStatus();
    if (result) setStatus(result);
  }, 1500);

  const available = status?.available ?? false;
  const running = status?.running ?? false;

  const start = async () => {
    setBusy(true);
    setStatus((await api.worldSplatStart()) ?? status);
    setBusy(false);
  };
  const stop = async () => {
    setBusy(true);
    setStatus((await api.worldSplatStop()) ?? status);
    setBusy(false);
  };
  const bootstrap = async () => {
    if (!selectedFlightId) {
      setMessage("Select a flight to bootstrap from.");
      return;
    }
    setBusy(true);
    setMessage("Running COLMAP cross-drone co-registration…");
    const result = await api.worldSplatBootstrapFlights([selectedFlightId]);
    if (result) setStatus(result);
    setMessage(result ? "Bootstrap complete." : "Bootstrap failed (see service logs).");
    setBusy(false);
  };
  const snapshot = async () => {
    const result = await downloadWorldSnapshot();
    setMessage(result.ok ? "Snapshot downloaded." : `Snapshot failed: ${result.error ?? ""}`);
  };

  const byDrone = status?.keyframesByDrone ?? {};

  return (
    <div className="world-view">
      <Panel
        title="Live World Model"
        right={
          <Pill tone={!available ? "danger" : running ? "ok" : "default"}>
            {!available ? "Unavailable" : running ? "Fusing" : "Idle"}
          </Pill>
        }
      >
        {!available && (
          <p className="note">
            gsplat / CUDA unavailable{status?.reason ? `: ${status.reason}` : ""}. Offline reconstruction still works.
          </p>
        )}
        <KeyValue
          entries={[
            { key: "Gaussians", value: status?.gaussians },
            { key: "Keyframes", value: status?.keyframes },
            { key: "Optimizer steps", value: status?.steps },
            { key: "Last loss", value: status?.lastLoss },
            { key: "Drones fused", value: Object.keys(byDrone).length || (status?.drones?.length ?? 0) },
          ]}
        />
        <div className="button-row">
          <Button variant="primary" onClick={start} disabled={busy || !available || running}>
            Start
          </Button>
          <Button onClick={stop} disabled={busy || !running}>
            Stop
          </Button>
          <Button onClick={bootstrap} disabled={busy || !available}>
            Bootstrap from flight
          </Button>
          <Button onClick={snapshot} disabled={!running}>
            Download .ply
          </Button>
        </div>
        {message && <p className="note">{message}</p>}
      </Panel>

      {Object.keys(byDrone).length > 0 && (
        <Panel title="Keyframes by drone">
          <KeyValue entries={Object.entries(byDrone).map(([drone, count]) => ({ key: drone, value: count }))} />
        </Panel>
      )}
    </div>
  );
}
