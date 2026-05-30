import { useCallback, useEffect, useState } from "react";
import { api } from "../../api/client";
import { Button, KeyValue, Panel, Pill } from "../primitives";
import type { SerialBridge } from "../../api/types";

const REPO_URL = "https://github.com/JacobFV/drone-control.git";

/**
 * Connections is organised around usable *radios*. The PC's own Wi-Fi interface
 * is deliberately NOT listed: joining a drone AP with it would drop the laptop's
 * internet connection (single-radio cards can't do both), so it isn't a usable
 * drone link. The real path is an ESP32 serial bridge that carries the Wi-Fi link
 * to one drone — each bridge gets its own container, and the help block guides
 * bringing one online.
 */
export function ConnectionsPanel() {
  const [bridges, setBridges] = useState<SerialBridge[]>([]);

  const loadBridges = useCallback(async () => {
    const res = await api.getSerialBridges();
    setBridges(res?.bridges ?? []);
  }, []);

  useEffect(() => {
    void loadBridges();
  }, [loadBridges]);

  return (
    <div className="panel-stack">
      <Panel title="Radios" right={<Pill>{bridges.length} bridge{bridges.length === 1 ? "" : "s"}</Pill>}>
        <p className="muted">
          The PC's built-in Wi-Fi radio isn't listed here on purpose. Joining a drone access point with it would
          disconnect the laptop from the internet — one radio can't hold both links. Use an ESP32 serial bridge instead:
          it carries the Wi-Fi link to a drone over USB while the PC keeps its network.
        </p>
      </Panel>

      {bridges.map((bridge) => (
        <BridgeRadio key={bridge.port} bridge={bridge} />
      ))}

      <EspHelp onRefresh={loadBridges} foundCount={bridges.length} />
    </div>
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
