import { useEffect, useRef } from "react";
import { useSession } from "../../store/SessionContext";
import { sessionFramePath } from "../../api/client";
import { TileFrame } from "./TileFrame";

/**
 * Screen-space segmentation overlay: the live camera frame with detection boxes
 * + instance-mask polygons drawn on top, from the session segmentation status.
 */
export function SegOverlayTile({ droneId }: { droneId: string }) {
  const { serviceBase, snapshot } = useSession();
  const imgRef = useRef<HTMLImageElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const detections = snapshot?.session.segmentation?.screen?.[droneId] ?? [];
  const detRef = useRef(detections);
  detRef.current = detections;

  useEffect(() => {
    if (!serviceBase) return;
    let raf = 0;
    let last = 0;
    const path = sessionFramePath(droneId);
    const tick = (t: number) => {
      if (t - last > 100 && imgRef.current) {
        last = t;
        imgRef.current.src = `${serviceBase.replace(/\/$/, "")}${path}?t=${Math.floor(t)}`;
      }
      drawOverlay(canvasRef.current, imgRef.current, detRef.current);
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [serviceBase, droneId]);

  return (
    <TileFrame
      id={`seg-${droneId}`}
      title={`Segmentation · ${droneId}`}
      badge={<span className="tile-count">{detections.length}</span>}
    >
      <div className="camera-body">
        <img ref={imgRef} alt={`seg ${droneId}`} className="camera-img" />
        <canvas ref={canvasRef} className="seg-canvas" />
      </div>
    </TileFrame>
  );
}

const PALETTE = ["#7fd1ff", "#ffd35a", "#8be0a0", "#f0a39d", "#c9a3ff", "#ff9f5a"];

function classColor(cls: string): string {
  let h = 0;
  for (let i = 0; i < cls.length; i += 1) h = (h * 31 + cls.charCodeAt(i)) % PALETTE.length;
  return PALETTE[h];
}

function drawOverlay(
  canvas: HTMLCanvasElement | null,
  img: HTMLImageElement | null,
  detections: { cls: string; score: number; bbox: number[]; polygon: number[][]; width: number; height: number }[],
): void {
  if (!canvas || !img) return;
  const rect = canvas.getBoundingClientRect();
  const w = Math.max(1, Math.floor(rect.width));
  const h = Math.max(1, Math.floor(rect.height));
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr;
    canvas.height = h * dpr;
  }
  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  for (const det of detections) {
    const sx = w / (det.width || w);
    const sy = h / (det.height || h);
    const color = classColor(det.cls);
    if (det.polygon.length > 2) {
      ctx.beginPath();
      det.polygon.forEach(([px, py], i) => {
        const x = px * w;
        const y = py * h;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.closePath();
      ctx.fillStyle = `${color}33`;
      ctx.fill();
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
    const [bx, by, bw, bh] = det.bbox;
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(bx * sx, by * sy, bw * sx, bh * sy);
    ctx.fillStyle = color;
    ctx.font = "11px ui-monospace, monospace";
    ctx.fillText(`${det.cls} ${(det.score * 100).toFixed(0)}%`, bx * sx + 3, by * sy + 12);
  }
}
