import React from "react";
import {
  AbsoluteFill,
  interpolate,
  useCurrentFrame,
  random,
  Easing,
} from "remotion";
import { C, MONO } from "./theme";

// ---- Background: faint engineering grid + vignette + animated grain ----
export const GridBG: React.FC<{ tint?: string }> = ({ tint }) => {
  const frame = useCurrentFrame();
  const drift = (frame * 0.15) % 40;
  return (
    <AbsoluteFill style={{ backgroundColor: C.bg }}>
      <AbsoluteFill
        style={{
          backgroundImage: `linear-gradient(${C.grid} 1px, transparent 1px), linear-gradient(90deg, ${C.grid} 1px, transparent 1px)`,
          backgroundSize: "40px 40px",
          backgroundPosition: `${drift}px ${drift}px`,
        }}
      />
      <AbsoluteFill
        style={{
          background:
            "radial-gradient(ellipse at 50% 45%, transparent 35%, rgba(0,0,0,0.72) 100%)",
        }}
      />
      {tint && (
        <AbsoluteFill style={{ backgroundColor: tint, mixBlendMode: "overlay" }} />
      )}
      <Grain />
    </AbsoluteFill>
  );
};

export const Grain: React.FC = () => {
  const frame = useCurrentFrame();
  const dots = new Array(60).fill(0).map((_, i) => {
    const seed = `${i}-${Math.floor(frame / 2)}`;
    return {
      x: random("x" + seed) * 1920,
      y: random("y" + seed) * 1080,
      o: random("o" + seed) * 0.05,
    };
  });
  return (
    <AbsoluteFill style={{ pointerEvents: "none" }}>
      {dots.map((d, i) => (
        <div
          key={i}
          style={{
            position: "absolute",
            left: d.x,
            top: d.y,
            width: 2,
            height: 2,
            background: "#fff",
            opacity: d.o,
          }}
        />
      ))}
    </AbsoluteFill>
  );
};

// scanline overlay used over footage
export const Scanlines: React.FC = () => (
  <AbsoluteFill
    style={{
      pointerEvents: "none",
      backgroundImage:
        "repeating-linear-gradient(0deg, rgba(0,0,0,0.18) 0px, rgba(0,0,0,0.18) 1px, transparent 1px, transparent 3px)",
      mixBlendMode: "multiply",
      opacity: 0.5,
    }}
  />
);

// ---- Typewriter mono text ----
export const Typewriter: React.FC<{
  text: string;
  startFrame: number;
  cps?: number; // chars per second
  style?: React.CSSProperties;
  cursor?: boolean;
}> = ({ text, startFrame, cps = 28, style, cursor = true }) => {
  const frame = useCurrentFrame();
  const elapsed = Math.max(0, frame - startFrame);
  const n = Math.min(text.length, Math.floor((elapsed / 30) * cps));
  const shown = text.slice(0, n);
  const blink = Math.floor(frame / 15) % 2 === 0;
  const done = n >= text.length;
  return (
    <span style={{ fontFamily: MONO, ...style }}>
      {shown}
      {cursor && (!done || blink) && (
        <span style={{ opacity: blink ? 1 : 0.25, color: C.green }}>▋</span>
      )}
    </span>
  );
};

// fade/slide-in wrapper
export const Reveal: React.FC<{
  at: number;
  dur?: number;
  y?: number;
  children: React.ReactNode;
  style?: React.CSSProperties;
}> = ({ at, dur = 14, y = 14, children, style }) => {
  const frame = useCurrentFrame();
  const p = interpolate(frame, [at, at + dur], [0, 1], {
    extrapolateLeft: "clamp",
    extrapolateRight: "clamp",
    easing: Easing.out(Easing.cubic),
  });
  return (
    <div
      style={{
        opacity: p,
        transform: `translateY(${(1 - p) * y}px)`,
        ...style,
      }}
    >
      {children}
    </div>
  );
};

// small label chip
export const Chip: React.FC<{
  children: React.ReactNode;
  color?: string;
  style?: React.CSSProperties;
}> = ({ children, color = C.dim, style }) => (
  <span
    style={{
      fontFamily: MONO,
      fontSize: 18,
      letterSpacing: 2,
      color,
      border: `1px solid ${color}`,
      borderRadius: 3,
      padding: "4px 10px",
      textTransform: "uppercase",
      ...style,
    }}
  >
    {children}
  </span>
);

// timecode / metadata corner overlays for the "real capture" feel
export const HudFrame: React.FC<{ label: string; tc: string }> = ({
  label,
  tc,
}) => {
  const frame = useCurrentFrame();
  const rec = Math.floor(frame / 15) % 2 === 0;
  return (
    <AbsoluteFill style={{ pointerEvents: "none", fontFamily: MONO }}>
      <div
        style={{
          position: "absolute",
          top: 40,
          left: 48,
          color: C.dim,
          fontSize: 18,
          letterSpacing: 2,
        }}
      >
        {label}
      </div>
      <div
        style={{
          position: "absolute",
          top: 40,
          right: 48,
          color: C.red,
          fontSize: 18,
          letterSpacing: 2,
          display: "flex",
          alignItems: "center",
          gap: 8,
        }}
      >
        <span style={{ opacity: rec ? 1 : 0.2 }}>●</span> REC {tc}
      </div>
      {/* corner ticks */}
      {[
        [40, 40, 1, 1],
        [40, 40, -1, 1],
        [40, 40, 1, -1],
        [40, 40, -1, -1],
      ].map(([mx, my, sx, sy], i) => (
        <div
          key={i}
          style={{
            position: "absolute",
            [sx > 0 ? "left" : "right"]: mx as number,
            [sy > 0 ? "top" : "bottom"]: my as number,
            width: 26,
            height: 26,
            borderTop: sy > 0 ? `2px solid ${C.dimmer}` : undefined,
            borderBottom: sy < 0 ? `2px solid ${C.dimmer}` : undefined,
            borderLeft: sx > 0 ? `2px solid ${C.dimmer}` : undefined,
            borderRight: sx < 0 ? `2px solid ${C.dimmer}` : undefined,
          }}
        />
      ))}
    </AbsoluteFill>
  );
};
