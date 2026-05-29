import { useEffect, useRef } from "react";
import { useSession } from "../../store/SessionContext";
import { api } from "../../api/client";
import { computeViewMatrix, projectPoint, type Vec3, type ViewMatrix } from "../../lib/pose3d";
import { TileFrame } from "./TileFrame";
import { useOrbit } from "./orbit";

/**
 * Interactive 3D tile: the back-projected depth point cloud. Polled from the
 * service (cheap, ~2 Hz) rather than carried on the live WS status, so the wall
 * stays lean. Each point is [x, y, z, r, g, b].
 */
export function PointCloudTile() {
  const { snapshot } = useSession();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { viewRef, reset } = useOrbit(canvasRef);
  const pointsRef = useRef<number[][]>([]);
  const available = snapshot?.session.depth?.available;
  const count = snapshot?.session.depth?.points ?? 0;

  useEffect(() => {
    let cancelled = false;
    const tick = async () => {
      const res = await api.getPointCloud(8000);  // latest N points
      if (!cancelled && res?.points) pointsRef.current = res.points;
    };
    void tick();
    const id = window.setInterval(tick, 500);
    return () => {
      cancelled = true;
      window.clearInterval(id);
    };
  }, []);

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
      for (const p of pointsRef.current) {
        const proj = projectPoint([p[0], p[1], p[2]] as Vec3, view, focal, w, h);
        if (!proj) continue;
        const size = Math.max(1, Math.min(3, 30 / proj[2]));
        ctx.fillStyle = `rgb(${p[3] | 0},${p[4] | 0},${p[5] | 0})`;
        ctx.fillRect(proj[0], proj[1], size, size);
      }
      raf = requestAnimationFrame(render);
    };
    raf = requestAnimationFrame(render);
    return () => cancelAnimationFrame(raf);
  }, [viewRef]);

  return (
    <TileFrame
      id="pointcloud"
      title="Point cloud (3D)"
      interactive
      badge={
        <span className="tile-actions">
          <span className="tile-count">{count}</span>
          <button type="button" className="tile-mini" onClick={reset}>
            reset
          </button>
        </span>
      }
    >
      <canvas ref={canvasRef} className="orbit-canvas" />
      {!available && <div className="tile-empty">depth model not installed</div>}
    </TileFrame>
  );
}

function drawGrid(ctx: CanvasRenderingContext2D, view: ViewMatrix, focal: number, w: number, h: number) {
  ctx.strokeStyle = "rgba(120,140,170,0.14)";
  ctx.lineWidth = 1;
  const span = 20;
  for (let i = -span; i <= span; i += 4) {
    seg(ctx, [i, -span, 0], [i, span, 0], view, focal, w, h);
    seg(ctx, [-span, i, 0], [span, i, 0], view, focal, w, h);
  }
}
function seg(ctx: CanvasRenderingContext2D, a: Vec3, b: Vec3, view: ViewMatrix, focal: number, w: number, h: number) {
  const pa = projectPoint(a, view, focal, w, h);
  const pb = projectPoint(b, view, focal, w, h);
  if (!pa || !pb) return;
  ctx.beginPath();
  ctx.moveTo(pa[0], pa[1]);
  ctx.lineTo(pb[0], pb[1]);
  ctx.stroke();
}
