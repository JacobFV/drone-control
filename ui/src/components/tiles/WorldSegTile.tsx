import { useEffect, useRef } from "react";
import { useSession } from "../../store/SessionContext";
import { computeViewMatrix, projectPoint, type Vec3, type ViewMatrix } from "../../lib/pose3d";
import { TileFrame } from "./TileFrame";
import { useOrbit } from "./orbit";
import type { WorldObject } from "../../api/types";

const PALETTE = ["#7fd1ff", "#ffd35a", "#8be0a0", "#f0a39d", "#c9a3ff", "#ff9f5a"];
function classColor(cls: string): string {
  let h = 0;
  for (let i = 0; i < cls.length; i += 1) h = (h * 31 + cls.charCodeAt(i)) % PALETTE.length;
  return PALETTE[h];
}

/** Interactive 3D tile: fused world-space segmented objects in the splat frame. */
export function WorldSegTile({ objects: objectsProp }: { objects?: WorldObject[] } = {}) {
  const { snapshot } = useSession();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { viewRef, reset } = useOrbit(canvasRef);
  const objects = objectsProp ?? snapshot?.session.segmentation?.world ?? [];
  const objRef = useRef<WorldObject[]>(objects);
  objRef.current = objects;

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
      for (const obj of objRef.current) {
        const p = projectPoint(obj.centroid as Vec3, view, focal, w, h);
        if (!p) continue;
        const color = classColor(obj.cls);
        const r = Math.max(3, Math.min(14, 60 / p[2]));
        ctx.fillStyle = `${color}cc`;
        ctx.beginPath();
        ctx.arc(p[0], p[1], r, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = color;
        ctx.font = "11px ui-monospace, monospace";
        ctx.fillText(`${obj.cls} ×${obj.count}`, p[0] + r + 2, p[1] + 3);
      }
      raf = requestAnimationFrame(render);
    };
    raf = requestAnimationFrame(render);
    return () => cancelAnimationFrame(raf);
  }, [viewRef]);

  return (
    <TileFrame
      id="world-seg"
      title="World objects (3D)"
      interactive
      badge={
        <span className="tile-actions">
          <span className="tile-count">{objects.length}</span>
          <button type="button" className="tile-mini" onClick={reset}>
            reset
          </button>
        </span>
      }
    >
      <canvas ref={canvasRef} className="orbit-canvas" />
      {objects.length === 0 && <div className="tile-empty">no world objects yet</div>}
    </TileFrame>
  );
}

function drawGrid(ctx: CanvasRenderingContext2D, view: ViewMatrix, focal: number, w: number, h: number) {
  ctx.strokeStyle = "rgba(120,140,170,0.16)";
  ctx.lineWidth = 1;
  const span = 20;
  for (let i = -span; i <= span; i += 2) {
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
