import { useCallback, useEffect, useRef, useState } from "react";
import { useSession } from "../../store/SessionContext";
import { api, openPath } from "../../api/client";
import { Button, Panel, Pill } from "../primitives";
import { FolderIcon, PlayIcon, StopIcon } from "../icons";
import type { DroneCommand, DroneDetail, RuntimeDrone, SimDrone } from "../../api/types";

/**
 * One container per drone: the command currently being sent to it, an e-stop /
 * resume toggle, a scrolling history of recent commands, and a link to open the
 * folder where that drone's files are written. Works for sim sessions (analytic
 * expert) and, where available, the live runtime.
 */
export function DronesPanel() {
  const { snapshot } = useSession();
  const session = snapshot?.session;
  const active = Boolean(session?.active);
  const isSim = session?.kind === "sim";

  const simDrones = (isSim ? ((session?.env as { drones?: SimDrone[] })?.drones ?? []) : []) as SimDrone[];
  const runtimeDrones = (snapshot?.runtime?.drones ?? []) as RuntimeDrone[];

  const droneIds = isSim ? simDrones.map((d) => d.droneId) : runtimeDrones.map((d) => d.droneId);

  // Poll per-drone detail (command history + record dir) while the tab is open.
  const [details, setDetails] = useState<Record<string, DroneDetail>>({});
  const idsKey = droneIds.join(",");
  const refreshDetails = useCallback(async () => {
    const ids = idsKey ? idsKey.split(",") : [];
    const results = await Promise.all(ids.map((id) => api.getDroneDetail(id)));
    setDetails((prev) => {
      const next = { ...prev };
      ids.forEach((id, i) => {
        const r = results[i];
        if (r) next[id] = r;
      });
      return next;
    });
  }, [idsKey]);

  useEffect(() => {
    if (!active) return;
    void refreshDetails();
    const t = setInterval(() => void refreshDetails(), 1500);
    return () => clearInterval(t);
  }, [active, refreshDetails]);

  if (!active) {
    return (
      <div className="panel-stack">
        <Panel title="Drones" right={<Pill>idle</Pill>}>
          <p className="muted">No active session. Start one to control and inspect individual drones here.</p>
        </Panel>
      </div>
    );
  }

  if (isSim) {
    return (
      <div className="panel-stack">
        {simDrones.map((d) => (
          <SimDroneCard key={d.droneId} drone={d} detail={details[d.droneId]} />
        ))}
      </div>
    );
  }

  return (
    <div className="panel-stack">
      {runtimeDrones.length === 0 && (
        <Panel title="Drones">
          <p className="muted">No drones in the live runtime yet.</p>
        </Panel>
      )}
      {runtimeDrones.map((d) => (
        <RuntimeDroneCard key={d.droneId} drone={d} detail={details[d.droneId]} />
      ))}
    </div>
  );
}

// -------------------------------------------------------------- sim drone card

function SimDroneCard({ drone, detail }: { drone: SimDrone; detail?: DroneDetail }) {
  const frozen = Boolean(drone.frozen);
  const [busy, setBusy] = useState(false);
  const toggle = async () => {
    setBusy(true);
    if (frozen) await api.droneRelease(drone.droneId);
    else await api.droneEstop(drone.droneId);
    setBusy(false);
  };
  return (
    <Panel
      title={drone.droneId}
      right={
        frozen ? <Pill tone="danger">e-stopped</Pill> : <Pill tone="ok">flying</Pill>
      }
      className="drone-card"
    >
      <ColorBar color={drone.color} />
      <CommandReadout command={drone.command} />
      <div className="drone-meta">
        {typeof drone.distance === "number" && <span className="muted">{drone.distance.toFixed(2)} m to goal</span>}
      </div>
      <div className="btn-row">
        <Button variant={frozen ? "primary" : "danger"} onClick={toggle} disabled={busy} className="with-icon">
          {frozen ? <PlayIcon size={14} /> : <StopIcon size={14} />}
          {frozen ? "Resume" : "E-stop"}
        </Button>
      </div>
      <CommandHistory entries={detail?.commands} />
      <FilesRow dir={detail?.dir} frameCount={detail?.frameCount} />
    </Panel>
  );
}

// ---------------------------------------------------------- runtime drone card

function RuntimeDroneCard({ drone, detail }: { drone: RuntimeDrone; detail?: DroneDetail }) {
  const armed = Boolean(drone.safety?.armed);
  const [busy, setBusy] = useState(false);
  const toggle = async () => {
    setBusy(true);
    if (armed) await api.runtimeDisarm(drone.droneId);
    else await api.runtimeArm(drone.droneId);
    setBusy(false);
  };
  const command: DroneCommand | null = drone.lastAction
    ? { roll: drone.lastAction.roll, pitch: drone.lastAction.pitch, throttle: drone.lastAction.throttle, yaw: drone.lastAction.yaw }
    : null;
  return (
    <Panel
      title={drone.droneId}
      right={armed ? <Pill tone="ok">armed</Pill> : <Pill tone="danger">disarmed</Pill>}
      className="drone-card"
    >
      <div className="drone-meta">
        <span className="muted">{drone.controller ?? "—"}</span>
        {drone.linkState && <span className="muted"> · {drone.linkState}</span>}
        {typeof drone.sent === "number" && <span className="muted"> · {drone.sent} sent</span>}
      </div>
      <CommandReadout command={command} />
      <div className="btn-row">
        <Button variant={armed ? "danger" : "primary"} onClick={toggle} disabled={busy} className="with-icon">
          {armed ? <StopIcon size={14} /> : <PlayIcon size={14} />}
          {armed ? "Disarm" : "Arm"}
        </Button>
      </div>
      <FilesRow dir={detail?.dir} frameCount={detail?.frameCount} />
    </Panel>
  );
}

// ----------------------------------------------------------------- subcomponents

function ColorBar({ color }: { color?: string }) {
  if (!color) return null;
  return <div className="drone-colorbar" style={{ background: color }} />;
}

const AXES: { key: keyof DroneCommand; label: string }[] = [
  { key: "roll", label: "Roll" },
  { key: "pitch", label: "Pitch" },
  { key: "throttle", label: "Thr" },
  { key: "yaw", label: "Yaw" },
];

/** Live stick command as four 0–255 bars (neutral 128 marked). */
function CommandReadout({ command }: { command?: DroneCommand | null }) {
  if (!command) return <p className="muted">No command yet.</p>;
  return (
    <div className="cmd-readout">
      {AXES.map(({ key, label }) => {
        const v = command[key] ?? 0;
        return (
          <div key={key} className="cmd-axis">
            <span className="cmd-axis-label">{label}</span>
            <span className="cmd-bar">
              <span className="cmd-bar-fill" style={{ width: `${(v / 255) * 100}%` }} />
              <span className="cmd-bar-center" />
            </span>
            <span className="cmd-axis-val mono">{v}</span>
          </div>
        );
      })}
    </div>
  );
}

function CommandHistory({ entries }: { entries?: DroneDetail["commands"] }) {
  const ref = useRef<HTMLOListElement>(null);
  useEffect(() => {
    if (ref.current) ref.current.scrollTop = ref.current.scrollHeight;
  }, [entries]);
  if (!entries || entries.length === 0) {
    return (
      <div className="cmd-history">
        <span className="section-label">Command history</span>
        <p className="muted">No commands logged yet.</p>
      </div>
    );
  }
  // Newest at the bottom (auto-scrolled into view).
  return (
    <div className="cmd-history">
      <span className="section-label">Command history</span>
      <ol className="cmd-log" ref={ref}>
        {entries.slice(-60).map((e, i) => (
          <li key={i} className="cmd-log-row mono">
            <span className="cmd-log-t">{e.t.toFixed(1)}s</span>
            {e.event ? (
              <span className={`cmd-log-event${e.event === "e-stop" ? " is-stop" : ""}`}>{e.event}</span>
            ) : (
              <span>
                r{e.roll} p{e.pitch} t{e.throttle} y{e.yaw}
              </span>
            )}
          </li>
        ))}
      </ol>
    </div>
  );
}

function FilesRow({ dir, frameCount }: { dir?: string | null; frameCount?: number }) {
  return (
    <div className="drone-files">
      <span className="section-label">Files</span>
      {dir ? (
        <>
          <code className="drone-files-path" title={dir}>{dir}</code>
          <div className="drone-files-meta">
            {typeof frameCount === "number" && <span className="muted">{frameCount} frames</span>}
            <Button onClick={() => openPath(dir)} className="with-icon">
              <FolderIcon size={14} /> Open folder
            </Button>
          </div>
        </>
      ) : (
        <p className="muted">Recording is off — no files on disk for this drone.</p>
      )}
    </div>
  );
}
