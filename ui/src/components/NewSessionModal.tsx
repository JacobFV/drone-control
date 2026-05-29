import { useEffect, useState } from "react";
import { useSession } from "../store/SessionContext";
import { api } from "../api/client";
import { Button, Field, SegmentedControl } from "./primitives";
import { CloseIcon } from "./icons";

interface SceneOption {
  id: string;
  name: string;
  kind: string;
}

/**
 * Session-creation lives here, not in the Flight tab — these are start-time
 * choices you don't edit mid-session, so the wall surfaces a modal first and
 * the Flight tab then shows the running config read-only.
 */
export function NewSessionModal() {
  const { newSessionOpen, setNewSessionOpen, setReviewSessionId, refreshState } = useSession();
  const [kind, setKind] = useState<"sim" | "real">("sim");
  const [scenes, setScenes] = useState<SceneOption[]>([]);
  const [scene, setScene] = useState("open_field");
  const [numDrones, setNumDrones] = useState(4);
  const [task, setTask] = useState("goto");
  const [rateHz, setRateHz] = useState(15);
  const [speed, setSpeed] = useState<"realtime" | "max">("realtime");
  const [worldModel, setWorldModel] = useState(true);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    if (!newSessionOpen) return;
    void api.getScenes().then((r) => {
      if (r?.scenes) setScenes(r.scenes);
    });
  }, [newSessionOpen]);

  if (!newSessionOpen) return null;

  const start = async () => {
    setBusy(true);
    const options =
      kind === "sim"
        ? { numDrones, task, scene, rateHz, maxSpeed: speed === "max", record: true }
        : { worldModel, record: true };
    const result = await api.sessionStart(kind, `${kind} session`, options);
    setBusy(false);
    if (result) {
      setReviewSessionId(null);
      setNewSessionOpen(false);
      await refreshState();
    }
  };

  return (
    <div className="modal-overlay" role="dialog" aria-modal="true" aria-label="New session">
      <div className="modal">
        <div className="modal-header">
          <h2>New flight session</h2>
          <button type="button" className="modal-close" aria-label="Close" onClick={() => setNewSessionOpen(false)}>
            <CloseIcon size={16} />
          </button>
        </div>

        <div className="modal-body">
          <Field label="Environment">
            <SegmentedControl
              ariaLabel="environment kind"
              value={kind}
              onChange={(v) => setKind(v)}
              options={[
                { value: "sim", label: "Simulated" },
                { value: "real", label: "Real" },
              ]}
            />
          </Field>

          {kind === "sim" ? (
            <>
              <Field label="Scene plan">
                <select value={scene} onChange={(e) => setScene(e.target.value)}>
                  {scenes.map((s) => (
                    <option key={s.id} value={s.id}>
                      {s.name} · {s.kind}
                    </option>
                  ))}
                </select>
              </Field>
              <Field label="Drones">
                <input type="number" min={1} max={16} value={numDrones} onChange={(e) => setNumDrones(Number(e.target.value))} />
              </Field>
              <Field label="Task">
                <select value={task} onChange={(e) => setTask(e.target.value)}>
                  <option value="goto">goto</option>
                  <option value="hover">hover</option>
                  <option value="formation">formation</option>
                </select>
              </Field>
              <Field label="Rate (Hz)">
                <input type="number" min={1} max={120} value={rateHz} onChange={(e) => setRateHz(Number(e.target.value))} />
              </Field>
              <Field label="Speed">
                <SegmentedControl
                  ariaLabel="sim speed"
                  value={speed}
                  onChange={(v) => setSpeed(v)}
                  options={[
                    { value: "realtime", label: "Realtime" },
                    { value: "max", label: "Max speed" },
                  ]}
                />
              </Field>
            </>
          ) : (
            <Field label="World model (splat)">
              <input type="checkbox" checked={worldModel} onChange={(e) => setWorldModel(e.target.checked)} />
            </Field>
          )}
        </div>

        <div className="modal-footer">
          <Button onClick={() => setNewSessionOpen(false)}>Cancel</Button>
          <Button variant="primary" onClick={start} disabled={busy}>
            {busy ? "Starting…" : "Start session"}
          </Button>
        </div>
      </div>
    </div>
  );
}
