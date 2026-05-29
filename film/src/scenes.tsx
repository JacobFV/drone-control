import React from "react";
import {
  AbsoluteFill,
  interpolate,
  spring,
  useCurrentFrame,
  useVideoConfig,
  OffthreadVideo,
  Img,
  staticFile,
  Easing,
} from "remotion";
import { C, MONO } from "./theme";
import { GridBG, Typewriter, Reveal, Chip, Scanlines, HudFrame } from "./components";

const clamp = { extrapolateLeft: "clamp" as const, extrapolateRight: "clamp" as const };

// ================================================================
// Shared: full-bleed footage backdrop with vignette / HUD / chip / title
// ================================================================
const Footage: React.FC<{
  src: string;
  isImage?: boolean;
  rate?: number;
  objectPosition?: string;
  hud: string;
  tc: string;
  chip: string;
  chipColor: string;
  title: React.ReactNode;
  titleAt?: number;
}> = ({ src, isImage, rate = 1, objectPosition, hud, tc, chip, chipColor, title, titleAt = 10 }) => {
  const frame = useCurrentFrame();
  const { durationInFrames } = useVideoConfig();
  const io = Math.min(
    interpolate(frame, [0, 14], [0, 1], clamp),
    interpolate(frame, [durationInFrames - 14, durationInFrames], [1, 0], clamp)
  );
  const scale = interpolate(frame, [0, durationInFrames], [1.05, 1.13]);
  return (
    <AbsoluteFill style={{ backgroundColor: C.bg }}>
      <AbsoluteFill style={{ opacity: io }}>
        {isImage ? (
          <Img src={staticFile("media/" + src)} style={{ width: "100%", height: "100%", objectFit: "cover", objectPosition, transform: `scale(${scale})` }} />
        ) : (
          <OffthreadVideo src={staticFile("media/" + src)} muted playbackRate={rate} style={{ width: "100%", height: "100%", objectFit: "cover", objectPosition, transform: `scale(${scale})`, filter: "saturate(0.92) contrast(1.05)" }} />
        )}
      </AbsoluteFill>
      <AbsoluteFill style={{ background: "linear-gradient(0deg, rgba(4,6,8,0.96) 0%, rgba(4,6,8,0.22) 40%, rgba(4,6,8,0.6) 100%)" }} />
      <Scanlines />
      <HudFrame label={hud} tc={tc} />
      <AbsoluteFill style={{ padding: "110px 110px", pointerEvents: "none" }}>
        <Reveal at={5}>
          <Chip color={chipColor}>{chip}</Chip>
        </Reveal>
        <Reveal at={titleAt} style={{ marginTop: 18, maxWidth: 1180 }}>
          <div style={{ fontFamily: MONO, fontSize: 42, color: C.ink, lineHeight: 1.2, textShadow: "0 2px 16px rgba(0,0,0,0.95)" }}>
            {title}
          </div>
        </Reveal>
      </AbsoluteFill>
    </AbsoluteFill>
  );
};

// ================================================================
// 1. HOOK — the $20 drone, on real footage, spec sheet
// ================================================================
export const SceneHook: React.FC = () => {
  const frame = useCurrentFrame();
  const spec: [string, string, boolean][] = [
    ["model", "E99 Pro · foldable-wing quad", false],
    ["camera", "dual · 640×384 RTP/JPEG", false],
    ["price", "$20 · AliExpress · free shipping", false],
  ];
  return (
    <AbsoluteFill style={{ backgroundColor: C.bg }}>
      <AbsoluteFill style={{ opacity: interpolate(frame, [0, 16], [0, 1], clamp) }}>
        <OffthreadVideo
          src={staticFile("media/drones_floor.mp4")}
          muted
          style={{ width: "100%", height: "100%", objectFit: "cover", transform: `scale(${interpolate(frame, [0, 180], [1.04, 1.12])})`, filter: "saturate(0.9) contrast(1.05)" }}
        />
      </AbsoluteFill>
      <AbsoluteFill style={{ background: "linear-gradient(90deg, rgba(4,6,8,0.92) 0%, rgba(4,6,8,0.4) 60%, rgba(4,6,8,0.7) 100%)" }} />
      <Scanlines />
      <HudFrame label="hardware · WIFI_8K / E99 airframe" tc="00:00" />
      <AbsoluteFill style={{ justifyContent: "center", padding: "0 120px" }}>
        <div style={{ fontFamily: MONO, fontSize: 58, color: C.ink, lineHeight: 1.12, maxWidth: 1300, textShadow: "0 2px 16px rgba(0,0,0,0.95)" }}>
          <Typewriter text="$20. it flies." startFrame={8} cps={16} />
        </div>
        <div style={{ marginTop: 40 }}>
          {spec.map(([k, v], i) => {
            const o = interpolate(frame, [56 + i * 9, 64 + i * 9], [0, 1], clamp);
            return (
              <div key={i} style={{ opacity: o, transform: `translateX(${(1 - o) * 16}px)`, display: "flex", gap: 22, marginBottom: 12, fontFamily: MONO, fontSize: 25 }}>
                <span style={{ color: C.dimmer, width: 110, textAlign: "right" }}>{k}</span>
                <span style={{ color: C.dim }}>{v}</span>
              </div>
            );
          })}
        </div>
      </AbsoluteFill>
    </AbsoluteFill>
  );
};

// ================================================================
// 2. TITLE — drone-control wordmark breath
// ================================================================
export const SceneTitle: React.FC = () => {
  const frame = useCurrentFrame();
  const { durationInFrames } = useVideoConfig();
  const io = Math.min(interpolate(frame, [0, 12], [0, 1], clamp), interpolate(frame, [durationInFrames - 12, durationInFrames], [1, 0], clamp));
  const bar = interpolate(frame, [6, 26], [0, 1], { ...clamp, easing: Easing.out(Easing.cubic) });
  return (
    <AbsoluteFill>
      <GridBG />
      <AbsoluteFill style={{ justifyContent: "center", alignItems: "center", opacity: io }}>
        <div style={{ width: 120, height: 2, background: C.amber, transform: `scaleX(${bar})`, marginBottom: 26 }} />
        <div style={{ fontFamily: MONO, fontSize: 96, color: C.ink, letterSpacing: 2, fontWeight: 600 }}>
          drone<span style={{ color: C.amber }}>-</span>control
        </div>
        <div style={{ fontFamily: MONO, fontSize: 22, color: C.dim, letterSpacing: 6, marginTop: 18 }}>
          A COORDINATION PROBLEM DISGUISED AS A DRONE APP
        </div>
      </AbsoluteFill>
    </AbsoluteFill>
  );
};

// ================================================================
// 3. COLLISION — the one-radio AP topology failure
// ================================================================
const DroneAP: React.FC<{ x: number; y: number; ssid: string; at: number }> = ({ x, y, ssid, at }) => {
  const frame = useCurrentFrame();
  const appear = spring({ frame: frame - at, fps: 30, config: { damping: 14 } });
  const ring = (frame - at) % 45;
  return (
    <g transform={`translate(${x},${y}) scale(${appear})`} opacity={appear}>
      {[0, 1, 2].map((i) => {
        const r = ((ring + i * 15) % 45) + 18;
        return <circle key={i} r={r} fill="none" stroke={C.amber} strokeWidth={1.5} opacity={interpolate(r, [18, 63], [0.5, 0])} />;
      })}
      <rect x={-26} y={-16} width={52} height={32} rx={4} fill={C.bg2} stroke={C.amber} strokeWidth={2} />
      <circle cx={0} cy={0} r={4} fill={C.amber} />
      <text x={0} y={48} fill={C.ink} fontFamily={MONO} fontSize={17} textAnchor="middle">{ssid}</text>
      <text x={0} y={70} fill={C.red} fontFamily={MONO} fontSize={15} textAnchor="middle">192.168.1.1</text>
    </g>
  );
};

export const SceneCollision: React.FC = () => {
  const frame = useCurrentFrame();
  return (
    <AbsoluteFill>
      <GridBG />
      <AbsoluteFill style={{ padding: "64px 110px" }}>
        <Reveal at={2}><Chip color={C.dim}>01 — the assumption</Chip></Reveal>
        <Reveal at={8} style={{ marginTop: 20 }}>
          <div style={{ fontFamily: MONO, fontSize: 44, color: C.ink, maxWidth: 1300 }}>
            every drone wants to be <span style={{ color: C.amber }}>its own access point.</span>
          </div>
        </Reveal>

        <svg width={1700} height={560} style={{ marginTop: 8 }}>
          <DroneAP x={330} y={200} ssid="WIFI_8K-0c5b90" at={26} />
          <DroneAP x={850} y={165} ssid="WIFI_8K-592b10" at={36} />
          <DroneAP x={1370} y={200} ssid="WIFI_8K-3e67bc" at={46} />
          <g transform="translate(850,450)">
            <rect x={-78} y={-36} width={156} height={72} rx={6} fill={C.bg2} stroke={C.cyan} strokeWidth={2} />
            <text x={0} y={-2} fill={C.cyan} fontFamily={MONO} fontSize={17} textAnchor="middle">your laptop</text>
            <text x={0} y={22} fill={C.dim} fontFamily={MONO} fontSize={14} textAnchor="middle">one radio · wlP9s9</text>
          </g>
          {[[330, 270], [850, 235], [1370, 270]].map(([x, y], i) => {
            const conflict = i !== 1;
            return (
              <line key={i} x1={x} y1={y} x2={850} y2={414} stroke={conflict ? C.red : C.green} strokeWidth={2}
                strokeDasharray={conflict ? "7 7" : undefined}
                opacity={interpolate(frame, [58, 70], [0, conflict ? 0.85 : 1], clamp)} />
            );
          })}
          {frame > 76 && [[590, 342], [1110, 342]].map(([x, y], i) => (
            <g key={i} opacity={interpolate(frame, [76, 86], [0, 1], clamp)}>
              <circle cx={x} cy={y} r={13} fill={C.bg} stroke={C.red} strokeWidth={2} />
              <text x={x} y={y + 6} fill={C.red} fontFamily={MONO} fontSize={18} textAnchor="middle">✕</text>
            </g>
          ))}
        </svg>

        <Reveal at={150} style={{ marginTop: -4 }}>
          <div style={{ fontFamily: MONO, fontSize: 26, color: C.dim, lineHeight: 1.5 }}>
            join one AP and you drop off everything else. <span style={{ color: C.red }}>three drones, one address, one radio.</span>
          </div>
        </Reveal>
      </AbsoluteFill>
    </AbsoluteFill>
  );
};

// ================================================================
// 4. INSIGHT — the pivot line
// ================================================================
export const SceneInsight: React.FC = () => {
  const frame = useCurrentFrame();
  return (
    <AbsoluteFill>
      <GridBG />
      <AbsoluteFill style={{ justifyContent: "center", padding: "0 160px" }}>
        <div style={{ width: 90, height: 2, background: C.amber, transform: `scaleX(${interpolate(frame, [0, 18], [0, 1], { ...clamp, easing: Easing.out(Easing.cubic) })})`, transformOrigin: "left", marginBottom: 34 }} />
        <div style={{ fontFamily: MONO, fontSize: 56, color: C.ink, lineHeight: 1.26, maxWidth: 1500 }}>
          <Reveal at={4}>so don't make</Reveal>
          <Reveal at={12}><span style={{ color: C.dim }}>the laptop solve</span></Reveal>
          <Reveal at={20}><span style={{ color: C.amber }}>a radio problem.</span></Reveal>
        </div>
        <Reveal at={44} style={{ marginTop: 34 }}>
          <span style={{ fontFamily: MONO, fontSize: 19, color: C.dimmer, letterSpacing: 2 }}>
            // move every association off the host
          </span>
        </Reveal>
      </AbsoluteFill>
    </AbsoluteFill>
  );
};

// ================================================================
// 5. BRIDGE — link chain + serial frame anatomy
// ================================================================
const Node: React.FC<{ x: number; w: number; label: string; sub: string; color: string; at: number }> = ({ x, w, label, sub, color, at }) => {
  const frame = useCurrentFrame();
  const o = interpolate(frame, [at, at + 10], [0, 1], clamp);
  return (
    <g opacity={o} transform={`translate(${x},120)`}>
      <rect x={-w / 2} y={-30} width={w} height={60} rx={5} fill={C.bg2} stroke={color} strokeWidth={2} />
      <text x={0} y={-2} fill={C.ink} fontFamily={MONO} fontSize={18} textAnchor="middle">{label}</text>
      <text x={0} y={18} fill={color} fontFamily={MONO} fontSize={13} textAnchor="middle">{sub}</text>
    </g>
  );
};

export const SceneBridge: React.FC = () => {
  const frame = useCurrentFrame();
  const stops = [330, 700, 1070, 1440];
  return (
    <AbsoluteFill>
      <GridBG />
      <AbsoluteFill style={{ padding: "64px 110px" }}>
        <Reveal at={2}><Chip color={C.cyan}>02 — the fix</Chip></Reveal>
        <Reveal at={8} style={{ marginTop: 20 }}>
          <div style={{ fontFamily: MONO, fontSize: 44, color: C.ink }}>
            give each drone <span style={{ color: C.cyan }}>its own radio.</span>
          </div>
        </Reveal>

        <svg width={1700} height={290} style={{ marginTop: 28 }}>
          <line x1={330} y1={120} x2={1440} y2={120} stroke={C.dimmer} strokeWidth={2} />
          {stops.slice(0, -1).map((s, i) => {
            const t = ((frame * 6) % 100) / 100;
            const px = s + (stops[i + 1] - s) * t;
            return <circle key={i} cx={px} cy={120} r={4} fill={C.green} opacity={frame > 40 ? 1 : 0} />;
          })}
          <Node x={330} w={150} label="control" sub="DroneAction" color={C.blue} at={16} />
          <Node x={700} w={170} label="USB-C serial" sub="921600 baud" color={C.green} at={24} />
          <Node x={1070} w={190} label="ESP32-C6" sub="WIFI_STA → AP" color={C.amber} at={32} />
          <Node x={1440} w={150} label="drone" sub="UDP :7099" color={C.red} at={40} />
        </svg>

        <Reveal at={56} style={{ marginTop: 4 }}>
          <div style={{ fontFamily: MONO, fontSize: 17, color: C.dim, marginBottom: 12, letterSpacing: 1 }}>esp32_drone_link · serial frame</div>
          <div style={{ display: "flex", gap: 4 }}>
            {[["DL", "magic", C.green], ["01", "ver", C.dim], ["TYPE", "msg", C.cyan], ["SEQ", "u16", C.dim], ["LEN", "u16", C.dim], ["…payload", "≤2048", C.amber], ["CRC", "16/CCITT", C.red]].map(([h, s, col], i) => (
              <Reveal at={58 + i * 4} key={i}>
                <div style={{ border: `1px solid ${col}`, borderRadius: 4, padding: "9px 15px", minWidth: 88, textAlign: "center", background: C.bg2 }}>
                  <div style={{ fontFamily: MONO, fontSize: 23, color: col as string }}>{h}</div>
                  <div style={{ fontFamily: MONO, fontSize: 12, color: C.dim, marginTop: 4 }}>{s}</div>
                </div>
              </Reveal>
            ))}
          </div>
        </Reveal>
        <Reveal at={96} style={{ marginTop: 22 }}>
          <div style={{ fontFamily: MONO, fontSize: 22, color: C.dim }}>
            flash once. <span style={{ color: C.green }}>it joins one AP and forwards every packet.</span>
          </div>
        </Reveal>
      </AbsoluteFill>
    </AbsoluteFill>
  );
};

// ================================================================
// 6. RIG — the physical hardware (hero photo)
// ================================================================
export const SceneRig: React.FC = () => (
  <Footage
    src="rig2.jpg"
    isImage
    objectPosition="center 36%"
    hud="bench · esp32-c6 ×3 · USB_V10 hub"
    tc="RIG"
    chip="the hardware"
    chipColor={C.amber}
    title={<>three <span style={{ color: C.amber }}>Seeed XIAO ESP32-C6</span> on one USB-C hub.</>}
  />
);

// ================================================================
// 7. TERMINAL — real linux bring-up: lsusb / serial / iw / nmcli
// ================================================================
type TLine = { k: "cmd" | "out" | "comment" | "ok" | "blank"; s: string };
export const SceneTerminal: React.FC = () => {
  const frame = useCurrentFrame();
  const lines: TLine[] = [
    { k: "cmd", s: "lsusb | grep -i espressif" },
    { k: "out", s: "Bus 003 Device 014: ID 303a:1001 Espressif USB JTAG/serial debug unit" },
    { k: "out", s: "Bus 003 Device 015: ID 303a:1001 Espressif USB JTAG/serial debug unit" },
    { k: "out", s: "Bus 003 Device 016: ID 303a:1001 Espressif USB JTAG/serial debug unit" },
    { k: "blank", s: "" },
    { k: "cmd", s: "ls /dev/serial/by-id/" },
    { k: "out", s: "usb-Espressif_..._58:E6:C5:1A:62:C4-if00" },
    { k: "out", s: "usb-Espressif_..._58:E6:C5:1A:66:68-if00" },
    { k: "out", s: "usb-Espressif_..._58:E6:C5:1A:F5:68-if00" },
    { k: "blank", s: "" },
    { k: "cmd", s: "iw dev wlP9s9 link" },
    { k: "out", s: "Not connected." },
    { k: "comment", s: "# host radio stays free — that's the whole point" },
    { k: "blank", s: "" },
    { k: "cmd", s: "nmcli -t -f SSID,SIGNAL dev wifi | grep WIFI_8K" },
    { k: "out", s: "WIFI_8K-0c5b90:71" },
    { k: "out", s: "WIFI_8K-592b10:66" },
    { k: "out", s: "WIFI_8K-3e67bc:63" },
    { k: "blank", s: "" },
    { k: "cmd", s: "python -m drone_control.bridge --scan" },
    { k: "ok", s: "ESP 62:C4 → WIFI_8K-3e67bc  READY  100 → 192.168.1.1:7099" },
    { k: "ok", s: "ESP 66:68 → WIFI_8K-592b10  READY" },
    { k: "ok", s: "ESP F5:68 → WIFI_8K-0c5b90  READY" },
  ];
  const shown = Math.max(0, Math.floor((frame - 6) / 6));
  const colorFor = (k: TLine["k"]) => (k === "cmd" ? C.ink : k === "comment" ? C.dimmer : k === "ok" ? C.green : C.dim);
  const blink = Math.floor(frame / 15) % 2 === 0;
  return (
    <AbsoluteFill>
      <GridBG />
      <AbsoluteFill style={{ justifyContent: "center", alignItems: "center" }}>
        <div style={{ width: 1520, background: "rgba(3,5,7,0.96)", border: `1px solid ${C.dimmer}`, borderRadius: 10, boxShadow: "0 40px 120px rgba(0,0,0,0.85)", overflow: "hidden" }}>
          <div style={{ display: "flex", alignItems: "center", gap: 10, padding: "12px 18px", borderBottom: `1px solid ${C.dimmer}`, background: "rgba(255,255,255,0.02)" }}>
            <div style={{ width: 12, height: 12, borderRadius: 12, background: "#ff5f56" }} />
            <div style={{ width: 12, height: 12, borderRadius: 12, background: "#ffbd2e" }} />
            <div style={{ width: 12, height: 12, borderRadius: 12, background: "#27c93f" }} />
            <div style={{ flex: 1, textAlign: "center", fontFamily: MONO, fontSize: 16, color: C.dim }}>brandonin@promxagh : ~/Code/drone-control</div>
          </div>
          <div style={{ padding: "22px 28px", minHeight: 580 }}>
            {lines.slice(0, shown + 1).map((l, i) => {
              if (l.k === "blank") return <div key={i} style={{ height: 13 }} />;
              const isLast = i === shown;
              return (
                <div key={i} style={{ fontFamily: MONO, fontSize: 22, lineHeight: 1.5, color: colorFor(l.k), whiteSpace: "pre" }}>
                  {l.k === "cmd" && <span style={{ color: C.green }}>$ </span>}
                  {l.k === "ok" && <span style={{ color: C.green }}>✓ </span>}
                  {l.s}
                  {isLast && l.k === "cmd" && <span style={{ color: C.green, opacity: blink ? 1 : 0.2 }}> ▋</span>}
                </div>
              );
            })}
          </div>
        </div>
      </AbsoluteFill>
      <Scanlines />
      <AbsoluteFill style={{ padding: "64px 110px", pointerEvents: "none" }}>
        <Reveal at={2}><Chip color={C.green}>03 — the bring-up</Chip></Reveal>
      </AbsoluteFill>
    </AbsoluteFill>
  );
};

// ================================================================
// 8. WIRE — the 9-byte control packet, live
// ================================================================
export const SceneWire: React.FC = () => {
  const frame = useCurrentFrame();
  const rows = [
    ["→ 03 66 80 80 80 80 00 00 99", "neutral"],
    ["→ 03 66 80 80 A8 80 00 28 99", "throttle 168"],
    ["→ 03 66 80 80 B0 80 00 30 99", "throttle 176  ↑ spin"],
    ["→ 01 01", "keepalive"],
  ];
  return (
    <AbsoluteFill>
      <GridBG />
      <AbsoluteFill style={{ padding: "64px 110px", justifyContent: "center" }}>
        <Reveal at={2}><Chip color={C.green}>04 — the wire</Chip></Reveal>
        <Reveal at={8} style={{ marginTop: 16 }}>
          <div style={{ fontFamily: MONO, fontSize: 38, color: C.ink }}>
            control is <span style={{ color: C.green }}>nine bytes</span> and a checksum.
          </div>
        </Reveal>
        <div style={{ marginTop: 28, border: `1px solid ${C.dimmer}`, borderRadius: 8, padding: "22px 28px", background: "rgba(0,0,0,0.4)", maxWidth: 980 }}>
          {rows.map(([hex, note], i) => {
            const o = interpolate(frame, [16 + i * 9, 22 + i * 9], [0, 1], clamp);
            return (
              <div key={i} style={{ opacity: o, display: "flex", justifyContent: "space-between", fontFamily: MONO, fontSize: 26, marginBottom: 14 }}>
                <span style={{ color: C.green }}>{hex}</span>
                <span style={{ color: C.dim }}>{note}</span>
              </div>
            );
          })}
        </div>
        <Reveal at={60} style={{ marginTop: 22 }}>
          <div style={{ fontFamily: MONO, fontSize: 22, color: C.dim }}>
            three bridges · <span style={{ color: C.green }}>sent 21 / 23 / 25 · errors 0</span>
          </div>
        </Reveal>
      </AbsoluteFill>
    </AbsoluteFill>
  );
};

// ================================================================
// 9. DRONES — the dumb fleet (footage), "they just take sticks"
// ================================================================
export const SceneDrones: React.FC = () => (
  <Footage
    src="drone_desk.mp4"
    hud="hardware · stick passthrough · disarmed"
    tc="00:04"
    chip="the fleet"
    chipColor={C.red}
    title={<>the drones stay dumb. <span style={{ color: C.red }}>they just take sticks.</span></>}
  />
);

// ================================================================
// 10. BRAINS — three-layer stack diagram
// ================================================================
export const SceneBrains: React.FC = () => {
  const frame = useCurrentFrame();
  const layers: [string, string, string, number][] = [
    ["VLM", "swarm coordinator · low frequency", C.amber, 14],
    ["VLA", "batched observation → action loops", C.cyan, 40],
    ["control policy", "one per drone · bounded actions", C.blue, 66],
    ["safety envelope", "clamp · heartbeat 0.75s · stop", C.green, 92],
  ];
  return (
    <AbsoluteFill>
      <GridBG />
      <AbsoluteFill style={{ padding: "64px 110px", justifyContent: "center" }}>
        <Reveal at={2}><Chip color={C.amber}>05 — the brains</Chip></Reveal>
        <Reveal at={6} style={{ marginTop: 16, marginBottom: 26 }}>
          <div style={{ fontFamily: MONO, fontSize: 38, color: C.ink }}>
            dumb drones. <span style={{ color: C.amber }}>smart host.</span> three layers.
          </div>
        </Reveal>
        {layers.map(([name, sub, col, at], i) => {
          const o = interpolate(frame, [at, at + 12], [0, 1], clamp);
          return (
            <div key={i} style={{ opacity: o, transform: `translateY(${(1 - o) * 16}px)`, marginLeft: i * 70, marginBottom: 14, display: "flex", alignItems: "center", gap: 18, border: `1px solid ${col}`, borderLeft: `4px solid ${col}`, borderRadius: 7, padding: "16px 22px", background: C.bg2, maxWidth: 1100 }}>
              <span style={{ fontFamily: MONO, fontSize: 26, color: col, minWidth: 230 }}>{name}</span>
              <span style={{ fontFamily: MONO, fontSize: 19, color: C.dim }}>{sub}</span>
              {i < 3 && o > 0.9 && <span style={{ marginLeft: "auto", color: C.dimmer, fontFamily: MONO, fontSize: 26 }}>↓</span>}
            </div>
          );
        })}
      </AbsoluteFill>
    </AbsoluteFill>
  );
};

// ================================================================
// 11. STACK — the running app, live (footage)
// ================================================================
export const SceneStack: React.FC = () => (
  <Footage
    src="app_running.mp4"
    rate={1.3}
    hud="drone-control · live · 20 Hz control · 1 Hz coordinator"
    tc="RUN"
    chip="running"
    chipColor={C.cyan}
    title={<>the station, live — <span style={{ color: C.cyan }}>camera, state, control</span> in one loop.</>}
  />
);

// ================================================================
// 12. TRAJECTORY — estimated flight path reconstruction (footage)
// ================================================================
export const SceneTrajectory: React.FC = () => (
  <Footage
    src="trajectory.mp4"
    rate={1.2}
    hud="perception · estimated trajectory · 126 keyframes"
    tc="EST"
    chip="perception"
    chipColor={C.green}
    title={<>and perception <span style={{ color: C.green }}>closes the loop</span> — pose from raw frames.</>}
  />
);

// ================================================================
// 13. FINALE — close over trajectory footage
// ================================================================
export const SceneFinale: React.FC = () => {
  const frame = useCurrentFrame();
  const { durationInFrames } = useVideoConfig();
  const fadeIn = interpolate(frame, [0, 18], [0, 1], clamp);
  const fadeOut = interpolate(frame, [durationInFrames - 28, durationInFrames - 2], [1, 0], clamp);
  return (
    <AbsoluteFill style={{ backgroundColor: C.bg }}>
      <AbsoluteFill style={{ opacity: fadeIn }}>
        <OffthreadVideo src={staticFile("media/traj_main.mp4")} muted style={{ width: "100%", height: "100%", objectFit: "cover", filter: "saturate(0.85) contrast(1.05)" }} />
      </AbsoluteFill>
      <AbsoluteFill style={{ background: "linear-gradient(90deg, rgba(6,8,10,0.93) 0%, rgba(6,8,10,0.5) 55%, rgba(6,8,10,0.85) 100%)" }} />
      <Scanlines />
      <HudFrame label="capture · 640×384 · RTP/JPEG :7070" tc="00:13" />
      <AbsoluteFill style={{ justifyContent: "center", padding: "0 110px", opacity: fadeOut }}>
        <Reveal at={20}>
          <div style={{ fontFamily: MONO, fontSize: 64, color: C.ink, lineHeight: 1.18, textShadow: "0 2px 16px rgba(0,0,0,0.95)" }}>
            the robots were easy.
          </div>
        </Reveal>
        <Reveal at={40}>
          <div style={{ fontFamily: MONO, fontSize: 64, color: C.amber, lineHeight: 1.18, textShadow: "0 2px 16px rgba(0,0,0,0.95)" }}>
            the network wasn't.
          </div>
        </Reveal>
        <Reveal at={72} style={{ marginTop: 30 }}>
          <span style={{ fontFamily: MONO, fontSize: 22, color: C.dim, letterSpacing: 3 }}>
            drone-control · fleet behavior is an infrastructure problem
          </span>
        </Reveal>
      </AbsoluteFill>
    </AbsoluteFill>
  );
};
