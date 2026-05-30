import { useEffect, useRef, useState } from "react";
import { useSession } from "../../store/SessionContext";
import { sessionOmniscientPath } from "../../api/client";
import { computeViewMatrix } from "../../lib/pose3d";
import { TileFrame } from "./TileFrame";
import { useOrbit } from "./orbit";

/**
 * Omniscient god's-eye view of the SIMULATION world — a free orbiting camera
 * showing the whole scene plus every drone (colored, heading-tagged) and its
 * goal. This is ground truth, not what any drone sees, so it only exists for
 * simulated sessions (the caller gates on session.kind === "sim").
 */
export function OmniscientTile() {
  const { serviceBase } = useSession();
  const bodyRef = useRef<HTMLDivElement>(null);
  const imgRef = useRef<HTMLImageElement>(null);
  const { viewRef, reset } = useOrbit(bodyRef);
  const [ok, setOk] = useState(false);

  useEffect(() => {
    if (!serviceBase) return;
    let raf = 0;
    let last = 0;
    let pending = false;
    const path = sessionOmniscientPath();
    const tick = (t: number) => {
      // The god view is server-rendered (480x360); ~8 fps is plenty and keeps
      // the sim loop free for the drone cameras.
      if (!pending && t - last > 120 && imgRef.current) {
        const view = computeViewMatrix(viewRef.current);
        const params = new URLSearchParams({
          t: String(Math.floor(t)),
          eyeX: view.eye[0].toFixed(3),
          eyeY: view.eye[1].toFixed(3),
          eyeZ: view.eye[2].toFixed(3),
          targetX: view.target[0].toFixed(3),
          targetY: view.target[1].toFixed(3),
          targetZ: view.target[2].toFixed(3),
        });
        last = t;
        pending = true;
        imgRef.current.src = `${serviceBase.replace(/\/$/, "")}${path}?${params.toString()}`;
      }
      raf = requestAnimationFrame(tick);
    };
    const img = imgRef.current;
    const settle = () => {
      pending = false;
    };
    img?.addEventListener("load", settle);
    img?.addEventListener("error", settle);
    raf = requestAnimationFrame(tick);
    return () => {
      cancelAnimationFrame(raf);
      img?.removeEventListener("load", settle);
      img?.removeEventListener("error", settle);
    };
  }, [serviceBase, viewRef]);

  return (
    <TileFrame
      id="omniscient"
      title="Omniscient · sim world"
      interactive
      badge={
        <button type="button" className="tile-mini" onClick={reset}>
          reset
        </button>
      }
    >
      <div ref={bodyRef} className="camera-body orbit-surface">
        <img
          ref={imgRef}
          alt="omniscient sim world"
          className="camera-img"
          draggable={false}
          onLoad={() => setOk(true)}
          onError={() => setOk(false)}
        />
        {!ok && <div className="tile-empty">rendering world…</div>}
      </div>
    </TileFrame>
  );
}
