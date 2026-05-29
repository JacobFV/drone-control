import { useEffect, useState } from "react";
import { useStation } from "../store/StationContext";
import { api } from "../api/client";
import { Button, Field, KeyValue, Panel } from "./primitives";

export function SettingsDrawer() {
  const { setSettingsOpen, network, config, refreshConfig, refreshNetwork } = useStation();

  // Wi-Fi connect form.
  const [iface, setIface] = useState("");
  const [ssid, setSsid] = useState("");
  const [password, setPassword] = useState("");
  const [busy, setBusy] = useState(false);
  const [message, setMessage] = useState("");

  // Transport / IO form (seeded from current config).
  const transport = config?.manual;
  const [linkType, setLinkType] = useState("udp");
  const [ip, setIp] = useState("");
  const [port, setPort] = useState(7099);
  const [protocol, setProtocol] = useState("wifi_8k_prefixed_short");
  const [serialPort, setSerialPort] = useState("");

  useEffect(() => {
    setIface(network?.defaultInterface ?? "");
  }, [network]);

  useEffect(() => {
    if (!transport) return;
    setLinkType(transport.linkType ?? "udp");
    setIp(transport.ip ?? "");
    setPort(transport.port ?? 7099);
    setProtocol(transport.protocol ?? "wifi_8k_prefixed_short");
    setSerialPort(transport.serialPort ?? "");
  }, [transport]);

  const connect = async () => {
    setBusy(true);
    const result = await api.connectWifi(iface, ssid, password);
    setMessage(result?.ok ? "Connected." : "Connect failed.");
    await refreshNetwork();
    setBusy(false);
  };
  const reconnect = async () => {
    setBusy(true);
    await api.reconnectWifi(iface, password);
    await refreshNetwork();
    setBusy(false);
  };
  const saveTransport = async () => {
    setBusy(true);
    await api.manualConfig({ linkType, iface, ip, port, protocol, serialPort, ssid, password });
    await refreshConfig();
    setMessage("Transport saved.");
    setBusy(false);
  };

  return (
    <div className="drawer-backdrop" onClick={() => setSettingsOpen(false)}>
      <div className="drawer" onClick={(e) => e.stopPropagation()}>
        <div className="drawer-header">
          <h2 className="section-label">Settings</h2>
          <Button onClick={() => setSettingsOpen(false)}>Close</Button>
        </div>

        <Panel title="Wi-Fi">
          <Field label="Interface">
            <input value={iface} onChange={(e) => setIface(e.target.value)} placeholder="wlan0" />
          </Field>
          <Field label="SSID">
            <input value={ssid} onChange={(e) => setSsid(e.target.value)} />
          </Field>
          <Field label="Password">
            <input type="password" value={password} onChange={(e) => setPassword(e.target.value)} />
          </Field>
          <div className="button-row">
            <Button variant="primary" onClick={connect} disabled={busy || !iface || !ssid}>Connect</Button>
            <Button onClick={reconnect} disabled={busy || !iface}>Reconnect</Button>
          </div>
        </Panel>

        <Panel title="Transport / IO">
          <Field label="Link type">
            <select value={linkType} onChange={(e) => setLinkType(e.target.value)}>
              <option value="udp">UDP (direct)</option>
              <option value="esp_serial">ESP32 serial bridge</option>
            </select>
          </Field>
          <Field label="Drone IP">
            <input value={ip} onChange={(e) => setIp(e.target.value)} placeholder="192.168.169.1" />
          </Field>
          <Field label="Port">
            <input type="number" value={port} onChange={(e) => setPort(Number(e.target.value))} />
          </Field>
          <Field label="Control protocol">
            <input value={protocol} onChange={(e) => setProtocol(e.target.value)} />
          </Field>
          {linkType === "esp_serial" && (
            <Field label="Serial port">
              <input value={serialPort} onChange={(e) => setSerialPort(e.target.value)} placeholder="/dev/ttyACM0" />
            </Field>
          )}
          <div className="button-row">
            <Button variant="primary" onClick={saveTransport} disabled={busy}>Save transport</Button>
          </div>
        </Panel>

        <Panel title="Policy & runtime">
          <KeyValue
            entries={[
              { key: "Max throttle", value: config?.policy?.maxThrottle },
              { key: "Command Hz", value: config?.policy?.commandHz },
              { key: "Throttle slew / s", value: config?.policy?.throttleSlewPerSecond },
              { key: "Heartbeat timeout (s)", value: config?.policy?.heartbeatTimeoutSeconds },
              { key: "Dry run", value: config?.runtime?.dryRun ? "yes" : "no" },
              { key: "IO enabled", value: config?.runtime?.enableIo ? "yes" : "no" },
              { key: "Local VLA", value: config?.runtime?.localVlaConfigured ? "configured" : "—" },
              { key: "Internet VLM", value: config?.runtime?.internetVlmConfigured ? "configured" : "—" },
            ]}
          />
        </Panel>

        {message && <p className="note">{message}</p>}
      </div>
    </div>
  );
}
