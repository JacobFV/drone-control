import { useCallback, useEffect, useState } from "react";
import { useSession } from "../../store/SessionContext";
import { api } from "../../api/client";
import { Button, Field, KeyValue, Panel, Pill } from "../primitives";
import type { AccessPoint, Drone, SerialBridge, WifiInterface, RuntimeDrone } from "../../api/types";

const REPO_URL = "https://github.com/JacobFV/drone-control.git";

/**
 * Connections is organised around the conceptual model: the PC's own network is
 * fixed, so what matters is each usable *radio* — a PC Wi-Fi interface, or an
 * ESP32 serial bridge that carries the Wi-Fi link to one drone. Each radio gets
 * its own container; the help block at the bottom guides bringing an ESP32 online.
 */
export function ConnectionsPanel() {
  const { state, snapshot, refreshState, refreshNetwork } = useSession();
  const [interfaces, setInterfaces] = useState<WifiInterface[]>([]);
  const [bridges, setBridges] = useState<SerialBridge[]>([]);
  const [apsByIface, setApsByIface] = useState<Record<string, AccessPoint[]>>({});
  const [scanning, setScanning] = useState<Record<string, boolean>>({});
  const [target, setTarget] = useState<{ iface: string; ssid: string } | null>(null);
  const [password, setPassword] = useState("");
  const [connecting, setConnecting] = useState(false);

  const knownDrones = state?.drones ?? [];
  const runtimeDrones = snapshot?.runtime.drones ?? [];

  const scan = useCallback(async (iface: string, rescan: boolean) => {
    setScanning((s) => ({ ...s, [iface]: true }));
    if (rescan) await api.discoverDrones(iface); // register detected drone APs as known
    const res = await api.getAccessPoints(iface, rescan);
    if (res?.accessPoints) setApsByIface((m) => ({ ...m, [iface]: res.accessPoints }));
    if (rescan) await refreshState();
    setScanning((s) => ({ ...s, [iface]: false }));
  }, [refreshState]);

  const loadRadios = useCallback(async () => {
    const [ifaceRes, bridgeRes] = await Promise.all([api.getWifiInterfaces(), api.getSerialBridges()]);
    const found = ifaceRes?.interfaces ?? [];
    setInterfaces(found);
    setBridges(bridgeRes?.bridges ?? []);
    for (const iface of found) void scan(iface.name, false);
  }, [scan]);

  useEffect(() => {
    void loadRadios();
  }, [loadRadios]);

  const connect = async () => {
    if (!target) return;
    setConnecting(true);
    await api.connectWifi(target.iface, target.ssid, password);
    setPassword("");
    const iface = target.iface;
    setTarget(null);
    await Promise.all([loadRadios(), refreshNetwork()]);
    await scan(iface, false);
    setConnecting(false);
  };

  const hasRadios = interfaces.length > 0 || bridges.length > 0;

  return (
    <div className="panel-stack">
      {!hasRadios && (
        <Panel title="Radios">
          <p className="muted">No usable radios detected yet. Attach a Wi-Fi adapter or an ESP32 bridge over USB.</p>
        </Panel>
      )}

      {interfaces.map((iface) => (
        <WifiRadio
          key={iface.name}
          iface={iface}
          aps={apsByIface[iface.name] ?? []}
          scanning={Boolean(scanning[iface.name])}
          knownDrones={knownDrones}
          runtimeDrones={runtimeDrones}
          target={target?.iface === iface.name ? target.ssid : null}
          password={password}
          connecting={connecting}
          onScan={() => scan(iface.name, true)}
          onPickTarget={(ssid) => {
            setTarget({ iface: iface.name, ssid });
            setPassword("");
          }}
          onCancelTarget={() => setTarget(null)}
          onPassword={setPassword}
          onConnect={connect}
        />
      ))}

      {bridges.map((bridge) => (
        <BridgeRadio key={bridge.port} bridge={bridge} />
      ))}

      <EspHelp onRefresh={loadRadios} foundCount={bridges.length} />
    </div>
  );
}

function droneForSsid(ssid: string, known: Drone[]): Drone | undefined {
  if (!ssid) return undefined;
  return known.find((d) => (d.connection?.ssid as string | undefined) === ssid || d.name === ssid);
}

function WifiRadio({
  iface,
  aps,
  scanning,
  knownDrones,
  runtimeDrones,
  target,
  password,
  connecting,
  onScan,
  onPickTarget,
  onCancelTarget,
  onPassword,
  onConnect,
}: {
  iface: WifiInterface;
  aps: AccessPoint[];
  scanning: boolean;
  knownDrones: Drone[];
  runtimeDrones: RuntimeDrone[];
  target: string | null;
  password: string;
  connecting: boolean;
  onScan: () => void;
  onPickTarget: (ssid: string) => void;
  onCancelTarget: () => void;
  onPassword: (v: string) => void;
  onConnect: () => void;
}) {
  const connected = Boolean(iface.connection);
  const connectedDrone = droneForSsid(iface.connection, knownDrones);
  const linkState = connectedDrone
    ? runtimeDrones.find((r) => r.droneId === connectedDrone.id)?.linkState
    : undefined;

  return (
    <Panel
      title={`Wi-Fi radio · ${iface.name}`}
      right={<Pill tone={connected ? "ok" : "default"}>{connected ? "connected" : iface.state || "idle"}</Pill>}
    >
      <KeyValue
        entries={[
          { key: "Joined network", value: iface.connection || "—" },
          {
            key: "Drone",
            value: connectedDrone
              ? `${connectedDrone.name}${linkState ? ` (${linkState})` : ""}`
              : connected
                ? "unknown"
                : "—",
          },
        ]}
      />

      <div className="radio-scan-head">
        <span className="section-label">Detected</span>
        <Button onClick={onScan} disabled={scanning}>
          {scanning ? "Scanning…" : "Rescan"}
        </Button>
      </div>

      {aps.length === 0 ? (
        <p className="muted">{scanning ? "Scanning for networks…" : "Nothing detected yet — rescan to look."}</p>
      ) : (
        <ul className="ap-list">
          {aps.map((ap) => {
            const joined = ap.ssid === iface.connection && connected;
            const open = ap.security === "" || ap.security.toLowerCase() === "none";
            return (
              <li key={`${ap.ssid}-${ap.bssid}`} className={`ap-item${ap.likely_drone ? " is-drone" : ""}`}>
                <div className="ap-row">
                  <span className="ap-name">{ap.ssid || "(hidden)"}</span>
                  {ap.likely_drone && <Pill tone="recording">drone</Pill>}
                  <span className="ap-signal">{ap.signal}%</span>
                  {joined ? (
                    <Pill tone="ok">joined</Pill>
                  ) : (
                    <Button onClick={() => onPickTarget(ap.ssid)}>Connect</Button>
                  )}
                </div>
                {target === ap.ssid && !joined && (
                  <div className="ap-connect">
                    {!open && (
                      <Field label="Password">
                        <input value={password} type="password" autoFocus onChange={(e) => onPassword(e.target.value)} />
                      </Field>
                    )}
                    <div className="btn-row">
                      <Button variant="primary" onClick={onConnect} disabled={connecting}>
                        {connecting ? "Connecting…" : "Join"}
                      </Button>
                      <Button onClick={onCancelTarget} disabled={connecting}>
                        Cancel
                      </Button>
                    </div>
                  </div>
                )}
              </li>
            );
          })}
        </ul>
      )}
    </Panel>
  );
}

function BridgeRadio({ bridge }: { bridge: SerialBridge }) {
  const label = bridge.product || (bridge.is_esp ? "ESP32 bridge" : "Serial device");
  return (
    <Panel
      title={`ESP32 bridge · ${bridge.port.replace("/dev/", "")}`}
      right={<Pill tone={bridge.is_esp ? "ok" : "default"}>{bridge.is_esp ? "esp32" : "serial"}</Pill>}
    >
      <KeyValue
        entries={[
          { key: "Serial", value: bridge.serial || "—", mono: true },
          { key: "Device", value: label },
          { key: "Port", value: bridge.port, mono: true },
          ...(bridge.manufacturer ? [{ key: "Maker", value: bridge.manufacturer }] : []),
        ]}
      />
      <p className="muted">
        Serial bridge — carries the Wi-Fi link to one drone. The drone AP is assigned to this bridge when you start a
        real flight session.
      </p>
    </Panel>
  );
}

function EspHelp({ onRefresh, foundCount }: { onRefresh: () => void; foundCount: number }) {
  return (
    <details className="esp-help">
      <summary>Why isn’t my ESP32 showing up here?</summary>
      <div className="esp-help-body">
        <p className="muted">
          {foundCount > 0
            ? `${foundCount} ESP32 bridge${foundCount === 1 ? "" : "s"} detected. If one is missing, work through these:`
            : "No ESP32 bridges detected yet. Work through these steps:"}
        </p>
        <ol className="esp-steps">
          <li>
            <strong>Plug it in over USB</strong> and give it power. Bus-powered hubs often can’t supply enough current —
            use a <em>powered</em> USB hub or a direct port, or the board won’t enumerate.
          </li>
          <li>
            <strong>Flash the bridge firmware.</strong> An unflashed board still appears as a serial port but won’t act
            as a drone link. Install PlatformIO (<code>pip install platformio</code>), then:
            <pre className="esp-code">{`git clone ${REPO_URL}
cd drone-control/firmware/esp32_drone_link
# Seeed XIAO ESP32-C6 (default):
pio run -e seeed_xiao_esp32c6 -t upload --upload-port /dev/ttyACM0
# ESP32-S3 DevKit instead:
pio run -e esp32s3 -t upload --upload-port /dev/ttyACM0`}</pre>
          </li>
          <li>
            <strong>Grant serial access.</strong> On Linux the port is owned by the <code>dialout</code> group:
            <pre className="esp-code">{`sudo usermod -aG dialout $USER   # then log out and back in`}</pre>
          </li>
          <li>
            <strong>Rescan</strong> once it’s connected.
          </li>
        </ol>
        <div className="btn-row">
          <Button onClick={onRefresh}>Rescan devices</Button>
        </div>
      </div>
    </details>
  );
}
