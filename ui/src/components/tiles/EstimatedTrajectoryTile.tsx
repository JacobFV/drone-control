import { useEffect, useRef } from "react";
import { useSession } from "../../store/SessionContext";
import {
  computeViewMatrix,
  projectPoint,
  quatToMatrix,
  type Vec3,
  type ViewMatrix,
} from "../../lib/pose3d";
import { TileFrame } from "./TileFrame";
import { useOrbit } from "./orbit";
import type { EstimatedTrajectoryDrone, Pose, TrajectoryDrone } from "../../api/types";

/**
 * The drones' ESTIMATED world trajectory — where each drone *thinks* it is,
 * from monocular visual odometry on its camera stream alone. Drawn solid, over
 * a faint dashed ground-truth ("objective") track so the drift is visible. The
 * estimate is similarity-aligned to ground truth (sim); per-drone APE drift is
 * shown in the legend. This is the estimated counterpart to the Trajectories
 * tile, which renders the objective track.
 */
export function EstimatedTrajectoryTile() {
  const { snapshot } = useSession();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { viewRef, reset } = useOrbit(canvasRef);

  const est = snapshot?.session.estimatedTrajectories;
  const truth = snapshot?.session.trajectories ?? [];
  const drones = est?.drones ?? [];

  const dataRef = useRef<{ est: EstimatedTrajectoryDrone[]; truth: TrajectoryDrone[] }>({
    est: drones,
    truth,
  });
  dataRef.current = { est: drones, truth };

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
      const truthById = new Map(dataRef.current.truth.map((t) => [t.droneId, t]));
      for (const d of dataRef.current.est) {
        const gt = truthById.get(d.droneId);
        const color = d.color ?? gt?.color ?? "#7da2ff";
        if (gt) drawGhost(ctx, gt.poses, color, view, focal, w, h);
        drawEstimate(ctx, d.poses, color, view, focal, w, h);
      }
      raf = requestAnimationFrame(render);
    };
    raf = requestAnimationFrame(render);
    return () => cancelAnimationFrame(raf);
  }, [viewRef]);

  const reason = est && !est.available ? est.reason || "estimator unavailable" : null;

  return (
    <TileFrame
      id="estimated-trajectory"
      title="Estimated trajectory (VO)"
      interactive
      badge={
        <button type="button" className="tile-mini" onClick={reset}>
          reset
        </button>
      }
    >
      <canvas ref={canvasRef} className="orbit-canvas" />
      {reason ? (
        <div className="tile-empty">{reason}</div>
      ) : (
        <div className="orbit-legend">
          {drones.map((d) => (
            <span key={d.droneId} className="legend-item">
              <span className="legend-dot" style={{ background: d.color ?? "#7da2ff" }} />
              {d.droneId}
              {d.driftFinal != null ? (
                <span className="legend-meta"> · drift {d.driftFinal.toFixed(1)}m</span>
              ) : (
                <span className="legend-meta"> · {d.state}</span>
              )}
            </span>
          ))}
          {drones.length > 0 && (
            <span className="legend-item legend-note">solid = estimate · dashed = truth</span>
          )}
        </div>
      )}
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

/** Faint dashed ground-truth track, for comparison against the estimate. */
function drawGhost(
  ctx: CanvasRenderingContext2D,
  poses: Pose[],
  color: string,
  view: ViewMatrix,
  focal: number,
  w: number,
  h: number,
) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.globalAlpha = 0.4;
  ctx.lineWidth = 1.25;
  ctx.setLineDash([4, 4]);
  strokePath(ctx, poses, view, focal, w, h);
  ctx.restore();
}

function drawEstimate(
  ctx: CanvasRenderingContext2D,
  poses: Pose[],
  color: string,
  view: ViewMatrix,
  focal: number,
  w: number,
  h: number,
) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  strokePath(ctx, poses, view, focal, w, h);

  const last = poses[poses.length - 1];
  if (!last) return;
  const lp = projectPoint([last.x, last.y, last.z], view, focal, w, h);
  if (!lp) return;
  // Current estimated position.
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(lp[0], lp[1], 4, 0, Math.PI * 2);
  ctx.fill();
  // Estimated heading: the camera's forward optical axis (3rd column of the
  // orientation) projected from the current position — this is the "orientation"
  // half of the estimated pose, not just the path tangent.
  if (last.qw != null && last.qx != null && last.qy != null && last.qz != null) {
    const R = quatToMatrix(last.qw, last.qx, last.qy, last.qz);
    const fwd: Vec3 = [R[0][2], R[1][2], R[2][2]];
    const tip: Vec3 = [last.x + fwd[0] * 1.5, last.y + fwd[1] * 1.5, last.z + fwd[2] * 1.5];
    const tp = projectPoint(tip, view, focal, w, h);
    if (tp) {
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(lp[0], lp[1]);
      ctx.lineTo(tp[0], tp[1]);
      ctx.stroke();
    }
  }
}

function strokePath(
  ctx: CanvasRenderingContext2D,
  poses: Pose[],
  view: ViewMatrix,
  focal: number,
  w: number,
  h: number,
) {
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
