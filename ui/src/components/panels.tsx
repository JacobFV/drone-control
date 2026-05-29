import { useState } from "react";
import { useStation } from "../store/StationContext";
import { api } from "../api/client";
import { usePolling } from "../lib/usePolling";
import { Button, Field, KeyValue, Panel, Pill } from "./primitives";
import { upper } from "../lib/format";

// --------------------------------------------------------------------------- //
// Connect
// --------------------------------------------------------------------------- //

export function ConnectPanel() {
  const { network, selectedDrone, refreshState, refreshNetwork } = useStation();
  const [busy, setBusy] = useState(false);
  const iface = network?.defaultInterface ?? "";

  const discover = async () => {
    setBusy(true);
    await api.discoverDrones(iface);
    await refreshState();
    await refreshNetwork();
    setBusy(false);
  };

  const conn = selectedDrone?.connection;
  return (
    <Panel title="Connect" right={<Pill tone={selectedDrone ? "ok" : "default"}>{selectedDrone ? "Selected" : "None"}</Pill>}>
      <KeyValue
        entries={[
          { key: "Default interface", value: iface, mono: true },
          { key: "Wi-Fi interfaces", value: network?.interfaces?.length },
          { key: "Single-radio", value: network?.singleWifiLikely ? "yes" : "no" },
          { key: "SSID", value: conn?.ssid, mono: true },
          { key: "Drone IP", value: conn?.ip, mono: true },
          { key: "Control", value: conn?.control, mono: true },
        ]}
      />
      <div className="button-row">
        <Button variant="primary" onClick={discover} disabled={busy || !iface}>
          {busy ? "Scanning…" : "Discover drones"}
        </Button>
      </div>
      {network?.notes && <p className="note">{network.notes}</p>}
      <p className="note">Wi-Fi credentials and transport are configured in Settings.</p>
    </Panel>
  );
}

// --------------------------------------------------------------------------- //
// Fly (runtime + manual control)
// --------------------------------------------------------------------------- //

const CONTROLLER_MODES = ["disabled", "manual", "autonomy", "vla", "batched_vla"];

export function FlyPanel() {
  const { selectedDroneId, selectedRuntimeDrone, runtimeStatus, refreshRuntime } = useStation();
  const armed = Boolean(selectedRuntimeDrone?.safety?.armed);

  // Heartbeat while armed (mirrors the original 250ms cadence).
  usePolling(
    () => {
      if (armed && selectedDroneId) void api.runtimeHeartbeat(selectedDroneId);
    },
    250,
    armed && Boolean(selectedDroneId),
  );

  const act = async (fn: () => Promise<unknown>) => {
    await fn();
    await refreshRuntime();
  };

  const setAxis = (axis: string, value: number) => {
    if (selectedDroneId) void api.runtimeAxes(selectedDroneId, { [axis]: value });
  };

  if (!selectedDroneId) {
    return (
      <Panel title="Fly">
        <p className="note">Select a drone to control.</p>
      </Panel>
    );
  }

  return (
    <>
      <Panel
        title="Runtime"
        right={<Pill tone={runtimeStatus?.running ? "ok" : "default"}>{runtimeStatus?.running ? "Running" : "Stopped"}</Pill>}
      >
        <Field label="Controller">
          <select
            value={selectedRuntimeDrone?.controller ?? "disabled"}
            onChange={(e) => act(() => api.runtimeSetController(selectedDroneId, e.target.value))}
          >
            {CONTROLLER_MODES.map((mode) => (
              <option key={mode} value={mode}>
                {upper(mode)}
              </option>
            ))}
          </select>
        </Field>
        <div className="button-row">
          <Button variant="primary" onClick={() => act(() => api.runtimeStart())}>Start</Button>
          <Button onClick={() => act(() => api.runtimeStop())}>Stop</Button>
        </div>
        <KeyValue
          entries={[
            { key: "Link", value: `${selectedRuntimeDrone?.linkType ?? "—"} · ${selectedRuntimeDrone?.linkState ?? "—"}` },
            { key: "Controller", value: selectedRuntimeDrone?.controller },
            { key: "Safety", value: selectedRuntimeDrone?.safety?.armed ? "armed" : "disarmed" },
            { key: "Max throttle", value: selectedRuntimeDrone?.constraints?.maxThrottle },
            { key: "Confidence", value: selectedRuntimeDrone?.observation?.confidence },
            { key: "Sent", value: selectedRuntimeDrone?.sent },
            { key: "Errors", value: selectedRuntimeDrone?.errors },
          ]}
        />
      </Panel>

      <Panel title="Manual control" right={<Pill tone={armed ? "ok" : "danger"}>{armed ? "Armed" : "Disarmed"}</Pill>}>
        <div className="button-row">
          <Button variant="primary" onClick={() => act(() => api.runtimeArm(selectedDroneId))}>Arm</Button>
          <Button onClick={() => act(() => api.runtimeDisarm(selectedDroneId))}>Disarm</Button>
          <Button variant="danger" onClick={() => act(() => api.runtimeStopDrone(selectedDroneId))}>Stop</Button>
          <Button onClick={() => act(() => api.runtimeClearFault(selectedDroneId))}>Clear fault</Button>
        </div>
        <ControlPad setAxis={setAxis} />
        <Field label="Throttle">
          <input
            type="range"
            min={0}
            max={255}
            defaultValue={128}
            onChange={(e) => setAxis("throttle", Number(e.target.value))}
          />
        </Field>
        {selectedRuntimeDrone?.safety?.faultReason && (
          <p className="note danger">Fault: {selectedRuntimeDrone.safety.faultReason}</p>
        )}
      </Panel>
    </>
  );
}

function ControlPad({ setAxis }: { setAxis: (axis: string, value: number) => void }) {
  const hold = (axis: string, value: number) => ({
    onPointerDown: () => setAxis(axis, value),
    onPointerUp: () => setAxis(axis, 128),
    onPointerLeave: () => setAxis(axis, 128),
  });
  return (
    <div className="control-pad">
      <button className="pad pad-pu" {...hold("pitch", 255)}>▲</button>
      <button className="pad pad-pd" {...hold("pitch", 0)}>▼</button>
      <button className="pad pad-rl" {...hold("roll", 0)}>◀</button>
      <button className="pad pad-rr" {...hold("roll", 255)}>▶</button>
      <button className="pad pad-yl" {...hold("yaw", 0)}>↺</button>
      <button className="pad pad-yr" {...hold("yaw", 255)}>↻</button>
      <button className="pad pad-stop" onPointerDown={() => { setAxis("roll", 128); setAxis("pitch", 128); setAxis("yaw", 128); }}>■</button>
    </div>
  );
}

// --------------------------------------------------------------------------- //
// Record
// --------------------------------------------------------------------------- //

export function RecordPanel() {
  const { selectedFlight, selectedFlightId, sessionStatus, refreshSession, refreshState, setSelectedRecordId, selectedRecordId } =
    useStation();
  const recording = Boolean(sessionStatus?.running);

  const toggleRecord = async () => {
    if (!selectedFlightId) return;
    if (recording) await api.sessionStop(selectedFlightId);
    else await api.sessionStart(selectedFlightId);
    await refreshSession();
  };

  const records = selectedFlight?.records ?? [];
  return (
    <Panel title="Record" right={<Pill tone={recording ? "recording" : "default"}>{recording ? "Recording" : "Idle"}</Pill>}>
      <KeyValue
        entries={[
          { key: "Frames", value: sessionStatus?.frames },
          { key: "Resolution", value: selectedFlight?.metrics?.resolution },
          { key: "Records", value: records.length },
        ]}
      />
      <div className="button-row">
        <Button variant={recording ? "danger" : "primary"} onClick={toggleRecord} disabled={!selectedFlightId}>
          {recording ? "Stop recording" : "Start recording"}
        </Button>
      </div>
      <div className="record-list">
        {records.map((record) => (
          <div key={record.id} className={`record-item${record.id === selectedRecordId ? " is-selected" : ""}`}>
            <button className="record-label" onClick={() => setSelectedRecordId(record.id)}>
              <span className="tree-name">{record.label || record.type}</span>
              <span className="tree-sub mono">{record.type}</span>
            </button>
            <div className="record-actions">
              <Button onClick={() => api.recordExport(record.id, "mp4")}>MP4</Button>
              <Button onClick={() => api.recordReveal(record.id).then(() => refreshState())}>Reveal</Button>
              {record.type === "gaussian-splat" && (
                <Button onClick={() => api.recordSplatViewer(record.id)}>View</Button>
              )}
            </div>
          </div>
        ))}
        {records.length === 0 && <p className="empty small">No records for this flight.</p>}
      </div>
    </Panel>
  );
}

// --------------------------------------------------------------------------- //
// Reconstruct
// --------------------------------------------------------------------------- //

export function ReconstructPanel() {
  const { selectedFlightId, reconstructionStatus, refreshReconstruction } = useStation();
  const [maxImages, setMaxImages] = useState(300);
  const [steps, setSteps] = useState(30000);
  const job = reconstructionStatus?.job;
  const toolsReady = reconstructionStatus?.tools?.ready ?? false;
  const active = Boolean(job?.active);

  const build = async () => {
    if (!selectedFlightId) return;
    await api.reconstructionStart(selectedFlightId, { maxImages, maxIterations: steps });
    await refreshReconstruction();
  };
  const stop = async () => {
    if (!selectedFlightId) return;
    await api.reconstructionStop(selectedFlightId);
    await refreshReconstruction();
  };

  return (
    <Panel
      title="Reconstruct"
      right={<Pill tone={active ? "ok" : toolsReady ? "default" : "danger"}>{job?.stage ?? (toolsReady ? "Ready" : "Tools missing")}</Pill>}
    >
      <Field label="Max images">
        <input type="number" value={maxImages} min={1} onChange={(e) => setMaxImages(Number(e.target.value))} />
      </Field>
      <Field label="Iterations">
        <input type="number" value={steps} min={1} onChange={(e) => setSteps(Number(e.target.value))} />
      </Field>
      <div className="button-row">
        <Button variant="primary" onClick={build} disabled={!selectedFlightId || active || !toolsReady}>
          Build splat
        </Button>
        <Button onClick={stop} disabled={!active}>Stop</Button>
        {reconstructionStatus?.latestSplatRecord && (
          <Button onClick={() => api.recordSplatViewer(reconstructionStatus.latestSplatRecord!.id)}>View</Button>
        )}
      </div>
      <KeyValue
        entries={[
          { key: "State", value: job?.state },
          { key: "Stage", value: job?.stage },
          { key: "Dataset", value: job?.datasetRecordId, mono: true },
          { key: "Splat", value: job?.splatRecordId, mono: true },
        ]}
      />
      {job?.error && <p className="note danger">{job.error}</p>}
      <p className="note">Offline per-flight splat. For a live, cross-drone world model use the World Model view.</p>
    </Panel>
  );
}

// --------------------------------------------------------------------------- //
// Simulation (always visible)
// --------------------------------------------------------------------------- //

const SIM_TASKS = ["goto", "hover", "formation"];

export function SimPanel() {
  const { setMainView, setDataSource } = useStation();
  const [status, setStatus] = useState<{ running?: boolean; step?: number; numDrones?: number } | null>(null);
  const [numDrones, setNumDrones] = useState(4);
  const [task, setTask] = useState("goto");

  usePolling(async () => {
    const result = await api.getSimStatus();
    if (result) setStatus(result);
  }, 1000);

  const running = Boolean(status?.running);
  const start = async () => {
    await api.simStart({ numDrones, task, rateHz: 15, render: true });
    setDataSource("sim");
    setMainView("cameras");
  };
  const stop = async () => {
    await api.simStop();
  };

  return (
    <Panel title="Simulation" right={<Pill tone={running ? "ok" : "default"}>{running ? "Running" : "Stopped"}</Pill>}>
      <Field label="Drones">
        <input type="number" min={1} max={10} value={numDrones} onChange={(e) => setNumDrones(Number(e.target.value))} />
      </Field>
      <Field label="Task">
        <select value={task} onChange={(e) => setTask(e.target.value)}>
          {SIM_TASKS.map((t) => (
            <option key={t} value={t}>{upper(t)}</option>
          ))}
        </select>
      </Field>
      <div className="button-row">
        <Button variant="primary" onClick={start} disabled={running}>Start sim</Button>
        <Button onClick={stop} disabled={!running}>Stop</Button>
      </div>
      <KeyValue
        entries={[
          { key: "Step", value: status?.step },
          { key: "Drones", value: status?.numDrones },
        ]}
      />
      <p className="note">Watch the swarm in the Cameras and Trajectories views (source: Sim).</p>
    </Panel>
  );
}

// --------------------------------------------------------------------------- //
// Swarm / VLA (always visible)
// --------------------------------------------------------------------------- //

const SWARM_MODES = ["disabled", "manual", "autonomy", "vla", "batched_vla"];

export function SwarmPanel() {
  const { runtimeStatus, refreshRuntime } = useStation();
  const batched = runtimeStatus?.batchedVla;
  const [mode, setMode] = useState("batched_vla");

  const apply = async () => {
    await api.runtimeSetAllControllers(mode);
    await refreshRuntime();
  };

  return (
    <Panel
      title="Swarm · Batched VLA"
      right={<Pill tone={batched?.active ? "ok" : "default"}>{batched?.active ? "Active" : "Inactive"}</Pill>}
    >
      <Field label="Swarm controller">
        <select value={mode} onChange={(e) => setMode(e.target.value)}>
          {SWARM_MODES.map((m) => (
            <option key={m} value={m}>{upper(m)}</option>
          ))}
        </select>
      </Field>
      <div className="button-row">
        <Button variant="primary" onClick={apply}>Apply to all drones</Button>
        <Button onClick={() => refreshRuntime()}>Refresh</Button>
      </div>
      <KeyValue
        entries={[
          { key: "Configured model", value: runtimeStatus?.batchedVlaConfigured ? "yes" : "neutral fallback" },
          { key: "Batches run", value: batched?.batches },
          { key: "Last batch size", value: batched?.lastBatchSize },
          { key: "Max wait (s)", value: batched?.maxWaitSeconds },
          { key: "Drones", value: runtimeStatus?.drones?.length },
        ]}
      />
    </Panel>
  );
}
