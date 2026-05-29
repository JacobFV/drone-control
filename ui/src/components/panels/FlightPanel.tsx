import { useState } from "react";
import { useSession } from "../../store/SessionContext";
import { api } from "../../api/client";
import { Button, KeyValue, Panel, Pill } from "../primitives";
import { PlusIcon } from "../icons";

export function FlightPanel() {
  const { snapshot, state, refreshState, setNewSessionOpen } = useSession();
  const session = snapshot?.session;
  const active = Boolean(session?.active);
  const [busy, setBusy] = useState(false);

  const stop = async () => {
    setBusy(true);
    await api.sessionStop();
    await refreshState();
    setBusy(false);
  };
  const setLiveSpeed = async (mode: "realtime" | "max") => {
    if (active) await api.sessionSpeed(mode);
  };

  const seg = session?.segmentation?.status;
  const env = (session?.env ?? {}) as Record<string, unknown>;
  const currentSession = state?.environments
    .flatMap((e) => e.sessions)
    .find((s) => s.id === session?.sessionId);

  if (!active) {
    return (
      <div className="panel-stack">
        <Panel title="Session" right={<Pill>idle</Pill>}>
          <p className="muted">No active flight session. Start one to begin streaming.</p>
          <Button variant="primary" onClick={() => setNewSessionOpen(true)} className="with-icon">
            <PlusIcon size={15} /> New session
          </Button>
        </Panel>
      </div>
    );
  }

  // Config is fixed at start time → read-only here.
  const configEntries = [
    { key: "Kind", value: session?.kind ?? "—" },
    ...(session?.kind === "sim"
      ? [
          { key: "Scene", value: (env.scene as string) ?? "—" },
          { key: "Task", value: (env.task as string) ?? "—" },
          { key: "Rate (Hz)", value: (env.rateHz as number) ?? "—" },
        ]
      : []),
    { key: "Drones", value: (session?.drones ?? []).length },
    { key: "World model", value: session?.worldModel?.running ? "running" : "off" },
  ];

  return (
    <div className="panel-stack">
      <Panel title="Session" right={<Pill tone="recording">{session?.recording ? "recording" : "live"}</Pill>}>
        <KeyValue
          entries={[
            { key: "Elapsed (s)", value: session?.elapsedSeconds ?? 0 },
            { key: "Speed", value: session?.speed ?? "—" },
          ]}
        />
        {session?.kind === "sim" && (
          <div className="btn-row">
            <Button
              variant={session?.speed === "realtime" ? "primary" : "default"}
              onClick={() => setLiveSpeed("realtime")}
            >
              Realtime
            </Button>
            <Button
              variant={session?.speed === "max" ? "primary" : "default"}
              onClick={() => setLiveSpeed("max")}
            >
              Max speed
            </Button>
          </div>
        )}
        <div className="btn-row">
          <Button variant="danger" onClick={stop} disabled={busy}>
            {busy ? "Stopping…" : "Stop session"}
          </Button>
        </div>
      </Panel>

      <Panel title="Configuration" right={<Pill>read-only</Pill>}>
        <KeyValue entries={configEntries} />
      </Panel>

      <Panel title="Drones">
        <ul className="roster">
          {(session?.drones ?? []).map((id) => (
            <li key={id} className="roster-item">
              <span className="roster-name">{id}</span>
              <span className="roster-meta">{session?.frameCounts?.[id] ?? 0} frames</span>
            </li>
          ))}
        </ul>
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
