import { useEffect, useRef } from "react";
import { useSession } from "../../store/SessionContext";
import { computeViewMatrix, projectPoint, type Vec3, type ViewMatrix } from "../../lib/pose3d";
import { TileFrame } from "./TileFrame";
import { useOrbit } from "./orbit";
import type { TrajectoryDrone } from "../../api/types";

/** Interactive 3D tile: each drone's believed trajectory in the world frame. */
export function TrajectoryTile() {
  const { snapshot } = useSession();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { viewRef, reset } = useOrbit(canvasRef);
  const tracks = snapshot?.session.trajectories ?? [];
  const tracksRef = useRef<TrajectoryDrone[]>(tracks);
  tracksRef.current = tracks;

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
      ctx.fillStyle = "#0a0d12";
      ctx.fillRect(0, 0, w, h);
      const view = computeViewMatrix(viewRef.current);
      const focal = Math.min(w, h) * 0.9;
      drawGrid(ctx, view, focal, w, h);
      for (const track of tracksRef.current) drawTrack(ctx, track, view, focal, w, h);
      raf = requestAnimationFrame(render);
    };
    raf = requestAnimationFrame(render);
    return () => cancelAnimationFrame(raf);
  }, [viewRef]);

  return (
    <TileFrame
      id="trajectory"
      title="Trajectories (3D)"
      interactive
      badge={
        <button type="button" className="tile-mini" onClick={reset}>
          reset
        </button>
      }
    >
      <canvas ref={canvasRef} className="orbit-canvas" />
      <div className="orbit-legend">
        {tracks.map((t) => (
          <span key={t.droneId} className="legend-item">
            <span className="legend-dot" style={{ background: t.color }} />
            {t.droneId}
          </span>
        ))}
      </div>
    </TileFrame>
  );
}

function drawGrid(ctx: CanvasRenderingContext2D, view: ViewMatrix, focal: number, w: number, h: number) {
  ctx.strokeStyle = "rgba(120,140,170,0.16)";
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

function drawTrack(
  ctx: CanvasRenderingContext2D,
  track: TrajectoryDrone,
  view: ViewMatrix,
  focal: number,
  w: number,
  h: number,
) {
  ctx.strokeStyle = track.color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  let started = false;
  for (const pose of track.poses) {
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
  const last = track.poses[track.poses.length - 1];
  if (last) {
    const lp = projectPoint([last.x, last.y, last.z], view, focal, w, h);
    if (lp) {
      ctx.fillStyle = track.color;
      ctx.beginPath();
      ctx.arc(lp[0], lp[1], 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }
  if (track.goal) {
    const g = projectPoint(track.goal as Vec3, view, focal, w, h);
    if (g) {
      ctx.strokeStyle = track.color;
      ctx.lineWidth = 1.5;
      ctx.strokeRect(g[0] - 5, g[1] - 5, 10, 10);
    }
  }
}

function line(ctx: CanvasRenderingContext2D, a: Vec3, b: Vec3, view: ViewMatrix, focal: number, w: number, h: number) {
  const pa = projectPoint(a, view, focal, w, h);
  const pb = projectPoint(b, view, focal, w, h);
  if (!pa || !pb) return;
  ctx.beginPath();
  ctx.moveTo(pa[0], pa[1]);
  ctx.lineTo(pb[0], pb[1]);
  ctx.stroke();
}
