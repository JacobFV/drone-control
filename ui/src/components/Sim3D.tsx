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
import type { Pose, TrajectoryDrone } from "../api/types";

export function Sim3D() {
  const { dataSource, selectedFlightId } = useStation();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const viewRef = useRef<OrbitView>({ ...DEFAULT_VIEW, target: [...DEFAULT_VIEW.target] });
  const tracksRef = useRef<TrajectoryDrone[]>([]);
  const [summary, setSummary] = useState({ drones: 0, points: 0, running: false });

  // Poll trajectories from the selected source (sim live, or real runtime; if the
  // real runtime has no pose track yet, fall back to the selected flight's track).
  useEffect(() => {
    let cancelled = false;
    const tick = async () => {
      if (dataSource === "sim") {
        const result = await api.getSimTrajectories();
        if (cancelled || !result) return;
        tracksRef.current = result.drones;
        setSummary({
          drones: result.drones.length,
          points: result.drones.reduce((n, d) => n + d.poses.length, 0),
          running: Boolean(result.running),
        });
        return;
      }
      const runtime = await api.getRuntimeTrajectories();
      let drones: TrajectoryDrone[] = runtime?.drones ?? [];
      const hasPoses = drones.some((d) => d.poses.length > 0);
      if (!hasPoses && selectedFlightId) {
        const track = await api.getPoseTrack(selectedFlightId, 0);
        if (track?.poses?.length) {
          drones = [{ droneId: selectedFlightId, color: "#7fd1ff", goal: null, poses: track.poses }];
        }
      }
      if (cancelled) return;
      tracksRef.current = drones;
      setSummary({
        drones: drones.length,
        points: drones.reduce((n, d) => n + d.poses.length, 0),
        running: Boolean(runtime?.running),
      });
    };
    void tick();
    const id = window.setInterval(tick, 500);
    return () => {
      cancelled = true;
      window.clearInterval(id);
    };
  }, [dataSource, selectedFlightId]);

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
      ctx.fillStyle = "#0b0e13";
      ctx.fillRect(0, 0, w, h);
      const view = computeViewMatrix(viewRef.current);
      const focal = Math.min(w, h) * 0.9;
      drawGrid(ctx, view, focal, w, h);
      for (const track of tracksRef.current) {
        drawTrajectory(ctx, track, view, focal, w, h);
      }
      raf = window.requestAnimationFrame(render);
    };
    raf = window.requestAnimationFrame(render);
    return () => window.cancelAnimationFrame(raf);
  }, []);

  // Orbit / pan / zoom.
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
      if (panning) panView(view, dx, dy, Math.min(canvas.clientWidth, canvas.clientHeight));
      else {
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
    if (dataSource === "real" && selectedFlightId) await api.computePoseTrack(selectedFlightId);
  };

  return (
    <div className="sim-view">
      <canvas ref={canvasRef} className="sim-canvas" />
      <div className="sim-overlay">
        <div className="sim-buttons">
          <Button onClick={resetView}>Reset view</Button>
          {dataSource === "real" && <Button onClick={recompute}>Recompute</Button>}
        </div>
        <KeyValue
          entries={[
            { key: "Source", value: dataSource === "sim" ? "simulation" : "real" },
            { key: "Drones", value: summary.drones },
            { key: "Track points", value: summary.points },
            { key: "Live", value: summary.running ? "yes" : "no" },
          ]}
        />
        <div className="traj-legend">
          {tracksRef.current.map((t) => (
            <span key={t.droneId} className="legend-item">
              <span className="legend-dot" style={{ background: t.color }} />
              {t.droneId}
            </span>
          ))}
        </div>
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
  track: TrajectoryDrone,
  view: ReturnType<typeof computeViewMatrix>,
  focal: number,
  w: number,
  h: number,
): void {
  const poses = track.poses;
  if (poses.length) {
    ctx.strokeStyle = track.color;
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
      } else ctx.lineTo(p[0], p[1]);
    }
    ctx.stroke();

    const last: Pose = poses[poses.length - 1];
    const lp = projectPoint([last.x, last.y, last.z], view, focal, w, h);
    if (lp) {
      ctx.fillStyle = track.color;
      ctx.beginPath();
      ctx.arc(lp[0], lp[1], 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  if (track.goal) {
    const g = projectPoint([track.goal[0], track.goal[1], track.goal[2]], view, focal, w, h);
    if (g) {
      ctx.strokeStyle = track.color;
      ctx.lineWidth = 1.5;
      ctx.strokeRect(g[0] - 5, g[1] - 5, 10, 10);
    }
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
