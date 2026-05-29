import { useState } from "react";
import { useSession } from "../../store/SessionContext";
import { api } from "../../api/client";
import { Button, Field, KeyValue, Panel, Pill } from "../primitives";

export function ConnectionsPanel() {
  const { state, network, snapshot, refreshState, refreshNetwork } = useSession();
  const [iface, setIface] = useState("");
  const [ssid, setSsid] = useState("");
  const [password, setPassword] = useState("");
  const [busy, setBusy] = useState(false);

  const interfaces = network?.interfaces ?? [];
  const effectiveIface = iface || network?.defaultInterface || interfaces[0]?.name || "";
  const runtimeDrones = snapshot?.runtime.drones ?? [];

  const discover = async () => {
    setBusy(true);
    await api.discoverDrones(effectiveIface);
    await refreshState();
    setBusy(false);
  };
  const connect = async () => {
    setBusy(true);
    await api.connectWifi(effectiveIface, ssid, password);
    await refreshNetwork();
    setBusy(false);
  };

  return (
    <div className="panel-stack">
      <Panel title="Network" right={<Pill>{network?.platform ?? "—"}</Pill>}>
        <KeyValue
          entries={[
            { key: "Default iface", value: network?.defaultInterface ?? "—" },
            { key: "Interfaces", value: interfaces.map((i) => i.name).join(", ") || "—" },
          ]}
        />
        {network?.notes && <p className="muted">{network.notes}</p>}
      </Panel>

      <Panel title="Connect to drone Wi-Fi">
        <div className="form-rows">
          <Field label="Interface">
            <input value={effectiveIface} onChange={(e) => setIface(e.target.value)} placeholder="wlan0" />
          </Field>
          <Field label="SSID">
            <input value={ssid} onChange={(e) => setSsid(e.target.value)} placeholder="WIFI_8K-…" />
          </Field>
          <Field label="Password">
            <input value={password} type="password" onChange={(e) => setPassword(e.target.value)} />
          </Field>
        </div>
        <div className="btn-row">
          <Button onClick={discover} disabled={busy || !effectiveIface}>
            Discover
          </Button>
          <Button variant="primary" onClick={connect} disabled={busy || !effectiveIface || !ssid}>
            Connect
          </Button>
        </div>
      </Panel>

      <Panel title="Known drones" right={<span className="muted">{state?.drones.length ?? 0}</span>}>
        <ul className="roster">
          {(state?.drones ?? []).map((d) => {
            const rt = runtimeDrones.find((r) => r.droneId === d.id);
            return (
              <li key={d.id} className="roster-item">
                <span className="roster-name">{d.name}</span>
                <span className="roster-meta">
                  {rt?.linkState ? <Pill tone={rt.linkState === "connected" ? "ok" : "default"}>{rt.linkState}</Pill> : d.status}
                </span>
              </li>
            );
          })}
        </ul>
      </Panel>

      <Panel title="Runtime drones (live)">
        {runtimeDrones.length === 0 ? (
          <p className="muted">No runtime drones configured.</p>
        ) : (
          <ul className="roster">
            {runtimeDrones.map((r) => (
              <li key={r.droneId} className="roster-item">
                <span className="roster-name">{r.droneId}</span>
                <span className="roster-meta btn-row">
                  <Button onClick={() => api.runtimeArm(r.droneId)}>Arm</Button>
                  <Button variant="danger" onClick={() => api.runtimeDisarm(r.droneId)}>
                    Disarm
                  </Button>
                </span>
              </li>
            ))}
          </ul>
        )}
      </Panel>
    </div>
  );
}
