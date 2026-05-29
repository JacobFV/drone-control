import { useState } from "react";
import { useSession } from "../../store/SessionContext";
import { api } from "../../api/client";
import { Button, Field, KeyValue, Panel, Pill, SegmentedControl } from "../primitives";

export function FlightPanel() {
  const { snapshot, state, refreshState } = useSession();
  const session = snapshot?.session;
  const active = Boolean(session?.active);

  const [kind, setKind] = useState<"sim" | "real">("sim");
  const [numDrones, setNumDrones] = useState(4);
  const [task, setTask] = useState("goto");
  const [rateHz, setRateHz] = useState(15);
  const [speed, setSpeed] = useState<"realtime" | "max">("realtime");
  const [worldModel, setWorldModel] = useState(true);
  const [busy, setBusy] = useState(false);

  const start = async () => {
    setBusy(true);
    const options =
      kind === "sim"
        ? { numDrones, task, rateHz, maxSpeed: speed === "max", record: true }
        : { worldModel, record: true };
    await api.sessionStart(kind, `${kind} session`, options);
    await refreshState();
    setBusy(false);
  };
  const stop = async () => {
    setBusy(true);
    await api.sessionStop();
    await refreshState();
    setBusy(false);
  };
  const setLiveSpeed = async (mode: "realtime" | "max") => {
    setSpeed(mode);
    if (active) await api.sessionSpeed(mode);
  };

  const seg = session?.segmentation?.status;
  const currentSession = state?.environments
    .flatMap((e) => e.sessions)
    .find((s) => s.id === session?.sessionId);

  return (
    <div className="panel-stack">
      <Panel title="Environment">
        <SegmentedControl
          ariaLabel="environment kind"
          value={kind}
          onChange={(v) => setKind(v)}
          options={[
            { value: "sim", label: "Simulated" },
            { value: "real", label: "Real" },
          ]}
        />
        {kind === "sim" ? (
          <div className="form-rows">
            <Field label="Drones">
              <input
                type="number"
                min={1}
                max={16}
                value={numDrones}
                disabled={active}
                onChange={(e) => setNumDrones(Number(e.target.value))}
              />
            </Field>
            <Field label="Task">
              <select value={task} disabled={active} onChange={(e) => setTask(e.target.value)}>
                <option value="goto">goto</option>
                <option value="hover">hover</option>
                <option value="formation">formation</option>
              </select>
            </Field>
            <Field label="Rate (Hz)">
              <input
                type="number"
                min={1}
                max={120}
                value={rateHz}
                disabled={active}
                onChange={(e) => setRateHz(Number(e.target.value))}
              />
            </Field>
            <Field label="Speed">
              <SegmentedControl
                ariaLabel="sim speed"
                value={speed}
                onChange={(v) => void setLiveSpeed(v)}
                options={[
                  { value: "realtime", label: "Realtime" },
                  { value: "max", label: "Max speed" },
                ]}
              />
            </Field>
          </div>
        ) : (
          <Field label="World model">
            <input type="checkbox" checked={worldModel} disabled={active} onChange={(e) => setWorldModel(e.target.checked)} />
          </Field>
        )}
        {active ? (
          <Button variant="danger" onClick={stop} disabled={busy}>
            Stop session
          </Button>
        ) : (
          <Button variant="primary" onClick={start} disabled={busy}>
            Start session
          </Button>
        )}
      </Panel>

      <Panel
        title="Session"
        right={active ? <Pill tone="recording">{session?.recording ? "recording" : "live"}</Pill> : <Pill>idle</Pill>}
      >
        <KeyValue
          entries={[
            { key: "Active", value: active },
            { key: "Kind", value: session?.kind ?? "—" },
            { key: "Speed", value: session?.speed ?? "—" },
            { key: "Elapsed (s)", value: session?.elapsedSeconds ?? 0 },
            { key: "Drones", value: (session?.drones ?? []).length },
            { key: "World model", value: session?.worldModel?.running ? "running" : "off" },
          ]}
        />
      </Panel>

      <Panel title="Drones">
        {(session?.drones ?? []).length === 0 ? (
          <p className="muted">No drones in the active session.</p>
        ) : (
          <ul className="roster">
            {(session?.drones ?? []).map((id) => {
              const frames = session?.frameCounts?.[id] ?? 0;
              return (
                <li key={id} className="roster-item">
                  <span className="roster-name">{id}</span>
                  <span className="roster-meta">{frames} frames</span>
                </li>
              );
            })}
          </ul>
        )}
      </Panel>

      <Panel title="Segmentation">
        <KeyValue
          entries={[
            { key: "Model", value: seg?.model ?? "—" },
            { key: "Available", value: seg?.available ?? false },
            { key: "World objects", value: session?.segmentation?.world?.length ?? 0 },
          ]}
        />
        {seg && !seg.available && <p className="muted">{seg.reason}</p>}
      </Panel>

      {currentSession && (
        <Panel title="Records" right={<span className="muted">{currentSession.records.length}</span>}>
          <ul className="record-list">
            {currentSession.records.map((r) => (
              <li key={r.id} className="record-item">
                <span className="record-source">{r.source}</span>
                <span className="record-label">{r.label}</span>
              </li>
            ))}
          </ul>
          {session?.sessionId && (
            <div className="btn-row">
              <Button onClick={() => api.computePoseTrack(session.sessionId!)}>Compute pose</Button>
              <Button onClick={() => api.reconstructionStart(session.sessionId!)}>Reconstruct</Button>
            </div>
          )}
        </Panel>
      )}
    </div>
  );
}
