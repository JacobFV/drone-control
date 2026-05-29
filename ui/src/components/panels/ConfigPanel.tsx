import { useSession } from "../../store/SessionContext";
import { KeyValue, Panel, Pill } from "../primitives";

export function ConfigPanel() {
  const { config, snapshot, transport } = useSession();
  const seg = snapshot?.session.segmentation?.status;
  const depth = snapshot?.session.depth;
  const runtime = (config?.runtime ?? {}) as Record<string, unknown>;
  const recon = config?.reconstruction ?? {};
  const camera = (config?.camera ?? {}) as Record<string, unknown>;
  const policy = (config?.policy ?? {}) as Record<string, unknown>;

  return (
    <div className="panel-stack">
      <Panel title="Transport" right={<Pill tone={transport === "ws" ? "ok" : "default"}>{transport}</Pill>}>
        <p className="muted">
          {transport === "ws"
            ? "Live state is pushed over WebSocket."
            : transport === "poll"
              ? "WebSocket unavailable — polling the service."
              : "Connecting…"}
        </p>
      </Panel>

      <Panel title="Runtime">
        <KeyValue
          entries={[
            { key: "Dry run", value: runtime.dryRun },
            { key: "Enable IO", value: runtime.enableIo },
            { key: "Control Hz", value: runtime.controlHz },
            { key: "Local VLA", value: runtime.localVlaConfigured },
            { key: "Internet VLM", value: runtime.internetVlmConfigured },
          ]}
        />
      </Panel>

      <Panel title="Segmentation model">
        <KeyValue
          entries={[
            { key: "Model", value: seg?.model ?? "—" },
            { key: "Available", value: seg?.available ?? false },
          ]}
        />
        {seg && !seg.available && <p className="muted">{seg.reason}</p>}
      </Panel>

      <Panel title="Depth model">
        <KeyValue
          entries={[
            { key: "Model", value: depth?.model ?? "—" },
            { key: "Available", value: depth?.available ?? false },
            { key: "Cloud points", value: depth?.points ?? 0 },
          ]}
        />
        {depth && !depth.available && <p className="muted">{depth.reason}</p>}
      </Panel>

      <Panel title="Reconstruction tools" right={<Pill tone={recon.ready ? "ok" : "danger"}>{recon.ready ? "ready" : "missing"}</Pill>}>
        <KeyValue entries={Object.entries((recon.tools as Record<string, unknown>) ?? {}).map(([k, v]) => ({ key: k, value: v, mono: true }))} />
      </Panel>

      <Panel title="Camera">
        <KeyValue entries={Object.entries(camera).map(([k, v]) => ({ key: k, value: v, mono: true }))} />
      </Panel>

      <Panel title="Manual policy">
        <KeyValue entries={Object.entries(policy).map(([k, v]) => ({ key: k, value: v }))} />
      </Panel>
    </div>
  );
}
