import React from "react";
import { AbsoluteFill, useCurrentFrame } from "remotion";
import { C, MONO } from "./theme";

// Narration track — sharp, conversational. Hard CUT in/out (no fade).
// Master timeline; scene durations in Film.tsx are sized to these beats.
type Cue = { from: number; to: number; text: string };
const S = (s: number) => Math.round(s * 30);

export const CUES: Cue[] = [
  // HOOK
  { from: S(0.4), to: S(3.6), text: "so — twenty bucks on AliExpress. it flies, it ships from shenzhen." },
  { from: S(3.7), to: S(6.0), text: "foldable wings, dual camera. good enough to start." },
  // (title card breath 6.0–8.0, silent)
  // THE CATCH
  { from: S(8.1), to: S(11.4), text: "the catch: its controller insists on being the Wi-Fi access point." },
  { from: S(11.5), to: S(14.0), text: "and your laptop has exactly one radio." },
  { from: S(14.1), to: S(16.6), text: "so it's the drone, or the internet. never both." },
  { from: S(16.7), to: S(19.4), text: "and definitely not three drones at once — they all answer to 192.168.1.1." },
  // THE FIX
  { from: S(19.6), to: S(22.4), text: "the fix is to stop making the laptop do it." },
  { from: S(22.5), to: S(25.6), text: "give every drone its own radio — a Seeed XIAO ESP32-C6." },
  { from: S(25.7), to: S(29.2), text: "flash it once; it joins one drone's AP and bridges packets over USB-C." },
  // THE BRING-UP
  { from: S(29.4), to: S(33.0), text: "lsusb. iw. nmcli. three boards, three serial ports." },
  { from: S(33.1), to: S(36.8), text: "now one machine holds three separate Wi-Fi networks at once." },
  // THE WIRE
  { from: S(37.0), to: S(40.4), text: "control is nine bytes, a CRC-16, and a keepalive. nothing dropped." },
  { from: S(40.5), to: S(43.6), text: "the drones stay dumb — they just take stick commands." },
  // THE BRAINS
  { from: S(43.8), to: S(46.8), text: "so the intelligence lives on the host, in three layers." },
  { from: S(46.9), to: S(50.0), text: "each drone runs its own control policy." },
  { from: S(50.1), to: S(53.2), text: "a VLA batches every observation-action loop together." },
  { from: S(53.3), to: S(56.4), text: "and a slow VLM sits on top, steering the whole swarm." },
  // PERCEPTION + CLOSE
  { from: S(56.6), to: S(59.8), text: "perception closes the loop — pose, recovered from raw frames." },
  { from: S(60.0), to: S(65.5), text: "that's drone-control. the robots were easy. the network wasn't." },
];

export const TOTAL_FRAMES = S(65.8);

export const SubtitleTrack: React.FC = () => {
  const frame = useCurrentFrame();
  const cue = CUES.find((c) => frame >= c.from && frame < c.to);
  if (!cue) return null;
  return (
    <AbsoluteFill style={{ justifyContent: "flex-end", alignItems: "center", pointerEvents: "none" }}>
      <div
        style={{
          marginBottom: 58,
          maxWidth: 1560,
          textAlign: "center",
          fontFamily: MONO,
          fontSize: 31,
          lineHeight: 1.4,
          color: C.ink,
          background: "rgba(4,6,8,0.74)",
          border: `1px solid ${C.dimmer}`,
          borderRadius: 6,
          padding: "13px 28px",
          textShadow: "0 2px 10px rgba(0,0,0,0.95)",
        }}
      >
        <span style={{ color: C.green, marginRight: 12 }}>›</span>
        {cue.text}
      </div>
    </AbsoluteFill>
  );
};
