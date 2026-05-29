import { useEffect, useState } from "react";
import { api, type CoordinatorConfigResult } from "../../api/client";
import { usePolling } from "../../lib/usePolling";
import { Button, Field, KeyValue, Panel, Pill } from "../primitives";

/**
 * High-level "brain": the LLM director that issues guidance tool calls at low
 * frequency. Configure the provider/model/key here (persisted server-side), set
 * the mission objective, and watch the tool calls it emits. No analytic
 * fallback — if no model is configured, high-level control is off.
 */
export function BrainPanel() {
  const [data, setData] = useState<CoordinatorConfigResult | null>(null);
  const [provider, setProvider] = useState("anthropic");
  const [model, setModel] = useState("claude-opus-4-8");
  const [baseUrl, setBaseUrl] = useState("");
  const [apiKey, setApiKey] = useState("");
  const [objective, setObjective] = useState("survey the area and keep safe spacing");
  const [busy, setBusy] = useState(false);
  const [loaded, setLoaded] = useState(false);

  const refresh = async () => {
    const result = await api.getCoordinatorConfig();
    if (result) {
      setData(result);
      if (!loaded) {
        setProvider(result.config.provider);
        setModel(result.config.model);
        setBaseUrl(result.config.baseUrl);
        setLoaded(true);
      }
    }
  };
  useEffect(() => {
    void refresh();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  usePolling(refresh, 3000);

  const save = async () => {
    setBusy(true);
    const body: Record<string, unknown> = { provider, model, baseUrl, temperature: data?.config.temperature ?? 0.2 };
    if (apiKey) body.apiKey = apiKey;
    await api.setCoordinatorConfig(body);
    setApiKey("");
    await refresh();
    setBusy(false);
  };

  const mission = data?.mission;
  const toolCalls = mission?.toolCalls ?? [];
  const configured = data?.config.configured;

  return (
    <div className="panel-stack">
      <Panel
        title="LLM director"
        right={<Pill tone={configured ? "ok" : "danger"}>{configured ? "configured" : "off"}</Pill>}
      >
        <div className="form-rows">
          <Field label="Provider">
            <select value={provider} onChange={(e) => setProvider(e.target.value)}>
              <option value="anthropic">Anthropic</option>
              <option value="openai">OpenAI-compatible</option>
            </select>
          </Field>
          <Field label="Model">
            <input value={model} onChange={(e) => setModel(e.target.value)} placeholder="claude-opus-4-8" />
          </Field>
          <Field label="API key">
            <input
              type="password"
              value={apiKey}
              onChange={(e) => setApiKey(e.target.value)}
              placeholder={data?.config.hasApiKey ? "•••• (set — leave blank to keep)" : "paste key"}
            />
          </Field>
          <Field label="Base URL">
            <input value={baseUrl} onChange={(e) => setBaseUrl(e.target.value)} placeholder="(default)" />
          </Field>
        </div>
        <Button variant="primary" onClick={save} disabled={busy}>
          {busy ? "Saving…" : "Save coordinator config"}
        </Button>
        {data?.lastError && <p className="muted">last error: {data.lastError}</p>}
      </Panel>

      <Panel title="Mission" right={<Pill>{mission?.state ?? "idle"}</Pill>}>
        <Field label="Objective">
          <input value={objective} onChange={(e) => setObjective(e.target.value)} />
        </Field>
        <div className="btn-row">
          <Button variant="primary" disabled={!configured} onClick={() => api.missionStart(objective)}>
            Start mission
          </Button>
          <Button variant="danger" onClick={() => api.missionStop()}>
            Stop
          </Button>
        </div>
        {!configured && <p className="muted">Configure a provider + API key to enable high-level control.</p>}
        {mission?.notes?.length ? <p className="muted">{mission.notes.join(" · ")}</p> : null}
      </Panel>

      <Panel title="Latest tool calls" right={<span className="muted">{toolCalls.length}</span>}>
        {toolCalls.length === 0 ? (
          <p className="muted">No guidance issued yet. The director runs ~1 call / 5s while a mission is active.</p>
        ) : (
          <ul className="record-list">
            {toolCalls.map((tc, i) => (
              <li key={i} className="record-item">
                <span className="record-source">{tc.name}</span>
                <span className="record-label">{JSON.stringify(tc.arguments)}</span>
              </li>
            ))}
          </ul>
        )}
        <KeyValue entries={[{ key: "Cadence", value: "~1 tool call / 5 s" }]} />
      </Panel>
    </div>
  );
}
