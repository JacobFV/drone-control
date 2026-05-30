import { useEffect, useRef } from "react";
import { DEFAULT_VIEW, panView, type OrbitView } from "../../lib/pose3d";

/**
 * Wire pointer-drag orbit, shift/right-drag pan, and wheel zoom onto a canvas.
 * Returns a ref holding the live OrbitView (mutated in place) and a reset fn.
 * Shared by the interactive 3D tiles (trajectory, world segmentation).
 */
export function useOrbit(canvasRef: React.RefObject<HTMLElement | null>) {
  const viewRef = useRef<OrbitView>({ ...DEFAULT_VIEW, target: [...DEFAULT_VIEW.target] });

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
      view.distance = Math.max(2, Math.min(800, view.distance * (1 + Math.sign(e.deltaY) * 0.1)));
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
  }, [canvasRef]);

  const reset = () => {
    viewRef.current = { ...DEFAULT_VIEW, target: [...DEFAULT_VIEW.target] };
  };

  return { viewRef, reset };
}
