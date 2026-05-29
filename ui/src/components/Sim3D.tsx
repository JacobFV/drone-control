import { useEffect, useRef, useState } from "react";
import { useStation } from "../store/StationContext";
import { api } from "../api/client";
import { Button, KeyValue } from "./primitives";
import {
  DEFAULT_VIEW,
  computeViewMatrix,
  panView,
  projectPoint,
  type OrbitView,
  type Vec3,
} from "../lib/pose3d";
import type { Pose, PoseStatus } from "../api/types";

export function Sim3D() {
  const { selectedFlightId } = useStation();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const viewRef = useRef<OrbitView>({ ...DEFAULT_VIEW, target: [...DEFAULT_VIEW.target] });
  const posesRef = useRef<Pose[]>([]);
  const sinceRef = useRef(0);
  const [status, setStatus] = useState<PoseStatus | null>(null);
  const [, force] = useState(0);

  // Reset accumulation when the flight changes.
  useEffect(() => {
    posesRef.current = [];
    sinceRef.current = 0;
    setStatus(null);
  }, [selectedFlightId]);

  // Poll the pose track.
  useEffect(() => {
    if (!selectedFlightId) return;
    let cancelled = false;
    const tick = async () => {
      const result = await api.getPoseTrack(selectedFlightId, sinceRef.current);
      if (cancelled || !result) return;
      if (result.status) setStatus(result.status);
      if (result.poses && result.poses.length) {
        posesRef.current = posesRef.current.concat(result.poses);
        const last = result.poses[result.poses.length - 1];
        if (typeof last.frameIndex === "number") sinceRef.current = last.frameIndex;
        force((n) => n + 1);
      }
    };
    void tick();
    const id = window.setInterval(tick, 500);
    return () => {
      cancelled = true;
      window.clearInterval(id);
    };
  }, [selectedFlightId]);

  // Draw loop.
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    let raf = 0;
    const render = () => {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const w = Math.max(1, Math.floor(rect.width));
      const h = Math.max(1, Math.floor(rect.height));
      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = "#0b0e13";
      ctx.fillRect(0, 0, w, h);

      const view = computeViewMatrix(viewRef.current);
      const focal = Math.min(w, h) * 0.9;

      drawGrid(ctx, view, focal, w, h);
      drawTrajectory(ctx, posesRef.current, view, focal, w, h);

      raf = window.requestAnimationFrame(render);
    };
    raf = window.requestAnimationFrame(render);
    return () => window.cancelAnimationFrame(raf);
  }, []);

  // Pointer interaction: drag to orbit, shift/right-drag to pan, wheel to zoom.
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    let dragging = false;
    let panning = false;
    let lastX = 0;
    let lastY = 0;

    const down = (e: PointerEvent) => {
      dragging = true;
      panning = e.shiftKey || e.button === 2;
      lastX = e.clientX;
      lastY = e.clientY;
      canvas.setPointerCapture(e.pointerId);
    };
    const move = (e: PointerEvent) => {
      if (!dragging) return;
      const dx = e.clientX - lastX;
      const dy = e.clientY - lastY;
      lastX = e.clientX;
      lastY = e.clientY;
      const view = viewRef.current;
      if (panning) {
        panView(view, dx, dy, Math.min(canvas.clientWidth, canvas.clientHeight));
      } else {
        view.yaw -= dx * 0.01;
        view.pitch = Math.max(-1.5, Math.min(1.5, view.pitch + dy * 0.01));
      }
    };
    const up = (e: PointerEvent) => {
      dragging = false;
      panning = false;
      try {
        canvas.releasePointerCapture(e.pointerId);
      } catch {
        /* ignore */
      }
    };
    const wheel = (e: WheelEvent) => {
      e.preventDefault();
      const view = viewRef.current;
      view.distance = Math.max(2, Math.min(500, view.distance * (1 + Math.sign(e.deltaY) * 0.1)));
    };
    const ctxMenu = (e: Event) => e.preventDefault();

    canvas.addEventListener("pointerdown", down);
    canvas.addEventListener("pointermove", move);
    canvas.addEventListener("pointerup", up);
    canvas.addEventListener("wheel", wheel, { passive: false });
    canvas.addEventListener("contextmenu", ctxMenu);
    return () => {
      canvas.removeEventListener("pointerdown", down);
      canvas.removeEventListener("pointermove", move);
      canvas.removeEventListener("pointerup", up);
      canvas.removeEventListener("wheel", wheel);
      canvas.removeEventListener("contextmenu", ctxMenu);
    };
  }, []);

  const resetView = () => {
    viewRef.current = { ...DEFAULT_VIEW, target: [...DEFAULT_VIEW.target] };
  };

  const recompute = async () => {
    if (!selectedFlightId) return;
    await api.computePoseTrack(selectedFlightId);
    posesRef.current = [];
    sinceRef.current = 0;
  };

  return (
    <div className="sim-view">
      <canvas ref={canvasRef} className="sim-canvas" />
      <div className="sim-overlay">
        <div className="sim-buttons">
          <Button onClick={recompute}>Recompute</Button>
          <Button onClick={resetView}>Reset view</Button>
        </div>
        <KeyValue
          entries={[
            { key: "State", value: status?.state ?? "—" },
            { key: "FPS", value: status?.fps },
            { key: "Keyframes", value: status?.keyframes },
            { key: "Poses", value: posesRef.current.length },
            { key: "Scale", value: status?.scaleLocked ? "metric" : "arbitrary" },
            { key: "Intrinsics", value: status?.intrinsicsSource ?? "—" },
          ]}
        />
      </div>
    </div>
  );
}

function drawGrid(
  ctx: CanvasRenderingContext2D,
  view: ReturnType<typeof computeViewMatrix>,
  focal: number,
  w: number,
  h: number,
): void {
  ctx.strokeStyle = "rgba(120,140,170,0.18)";
  ctx.lineWidth = 1;
  const span = 20;
  for (let i = -span; i <= span; i += 2) {
    line(ctx, [i, -span, 0], [i, span, 0], view, focal, w, h);
    line(ctx, [-span, i, 0], [span, i, 0], view, focal, w, h);
  }
  // Axes
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#e0524a";
  line(ctx, [0, 0, 0], [4, 0, 0], view, focal, w, h);
  ctx.strokeStyle = "#5ac46a";
  line(ctx, [0, 0, 0], [0, 4, 0], view, focal, w, h);
  ctx.strokeStyle = "#4a8fe0";
  line(ctx, [0, 0, 0], [0, 0, 4], view, focal, w, h);
}

function drawTrajectory(
  ctx: CanvasRenderingContext2D,
  poses: Pose[],
  view: ReturnType<typeof computeViewMatrix>,
  focal: number,
  w: number,
  h: number,
): void {
  if (poses.length === 0) return;
  ctx.strokeStyle = "#7fd1ff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  let started = false;
  for (const pose of poses) {
    const p = projectPoint([pose.x, pose.y, pose.z], view, focal, w, h);
    if (!p) {
      started = false;
      continue;
    }
    if (!started) {
      ctx.moveTo(p[0], p[1]);
      started = true;
    } else {
      ctx.lineTo(p[0], p[1]);
    }
  }
  ctx.stroke();

  const last = poses[poses.length - 1];
  const lp = projectPoint([last.x, last.y, last.z], view, focal, w, h);
  if (lp) {
    ctx.fillStyle = "#ffd35a";
    ctx.beginPath();
    ctx.arc(lp[0], lp[1], 4, 0, Math.PI * 2);
    ctx.fill();
  }
}

function line(
  ctx: CanvasRenderingContext2D,
  a: Vec3,
  b: Vec3,
  view: ReturnType<typeof computeViewMatrix>,
  focal: number,
  w: number,
  h: number,
): void {
  const pa = projectPoint(a, view, focal, w, h);
  const pb = projectPoint(b, view, focal, w, h);
  if (!pa || !pb) return;
  ctx.beginPath();
  ctx.moveTo(pa[0], pa[1]);
  ctx.lineTo(pb[0], pb[1]);
  ctx.stroke();
}
