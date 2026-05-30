import { useEffect, useState } from "react";
import { api, openExternal, type ModelEntry } from "../../api/client";
import { Button, Panel, Pill } from "../primitives";
import { CheckIcon, CloudDownloadIcon } from "../icons";

function fmtSize(bytes: number): string {
  if (!bytes) return "—";
  if (bytes < 1e6) return `${(bytes / 1e3).toFixed(0)} KB`;
  if (bytes < 1e9) return `${(bytes / 1e6).toFixed(1)} MB`;
  return `${(bytes / 1e9).toFixed(2)} GB`;
}

/**
 * VLA policy registry: download models from the Hugging Face Hub and choose the
 * active medium-frequency policy. There is no default — until one is selected,
 * the VLA tier is off (drones get no medium-level actions).
 */
export function ModelsPanel() {
  const [models, setModels] = useState<ModelEntry[]>([]);
  const [active, setActive] = useState<string | null>(null);
  const [busy, setBusy] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const refresh = async () => {
    const r = await api.getModels();
    if (r) {
      setModels(r.models);
      setActive(r.active);
    }
  };
  useEffect(() => {
    void refresh();
  }, []);

  const download = async (id: string) => {
    setBusy(id);
    setError(null);
    try {
      const r = await api.downloadModelOrThrow(id);
      setModels(r.models);
      setActive(r.active);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      setError(`Download failed for ${id}: ${message}`);
    }
    setBusy(null);
  };
  const select = async (id: string | null) => {
    setBusy(id ?? "none");
    const r = await api.selectModel(id);
    if (r) {
      setModels(r.models);
      setActive(r.active);
    }
    setBusy(null);
  };

  return (
    <div className="panel-stack">
      <Panel
        title="Active VLA policy"
        right={<Pill tone={active ? "ok" : "danger"}>{active ?? "none (medium tier off)"}</Pill>}
      >
        <p className="muted">
          The medium-frequency controller. No fallback — pick a policy to enable it. Selection persists.
        </p>
        {active && (
          <Button onClick={() => select(null)} disabled={busy !== null}>
            Disable (no policy)
          </Button>
        )}
      </Panel>

      {models.map((m) => (
        <Panel
          key={m.id}
          title={m.name}
          right={
            <span className="tile-actions">
              {m.active && <Pill tone="ok">active</Pill>}
              <Pill>{m.params}</Pill>
            </span>
          }
        >
          <p className="muted">{m.description}</p>
          <div className="model-meta">
            <span>{m.downloaded ? `on disk · ${fmtSize(m.sizeBytes)}` : "not downloaded"}</span>
            <button type="button" className="link-btn" onClick={() => openExternal(m.ghUrl)}>
              {m.hfRepo}
            </button>
          </div>
          <div className="btn-row">
            {!m.downloaded ? (
              <Button className="with-icon" onClick={() => download(m.id)} disabled={busy !== null}>
                <CloudDownloadIcon size={15} /> {busy === m.id ? "Downloading…" : "Download"}
              </Button>
            ) : (
              <Button className="with-icon" onClick={() => download(m.id)} disabled={busy !== null} title="Re-download / update">
                <CloudDownloadIcon size={15} /> Update
              </Button>
            )}
            <Button
              variant={m.active ? "default" : "primary"}
              className="with-icon"
              disabled={!m.downloaded || m.active || busy !== null}
              onClick={() => select(m.id)}
            >
              {m.active ? (
                <>
                  <CheckIcon size={15} /> Selected
                </>
              ) : (
                "Use this policy"
              )}
            </Button>
          </div>
        </Panel>
      ))}
      {error && (
        <Panel title="Error">
          <p className="muted">{error}</p>
        </Panel>
      )}
    </div>
  );
}
