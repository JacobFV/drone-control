import { useEffect, useRef, useState } from "react";
import { useSession } from "../../store/SessionContext";
import { sessionFramePath } from "../../api/client";
import { TileFrame } from "./TileFrame";

/**
 * Live camera tile. The active session exposes a single-JPEG endpoint per drone
 * (`/api/session/drones/{id}/frame`); we refresh it ~12fps with a cache-buster
 * for a smooth realtime feed without holding an MJPEG socket open per tile.
 */
export function CameraTile({ droneId, color }: { droneId: string; color?: string }) {
  const { serviceBase } = useSession();
  const imgRef = useRef<HTMLImageElement>(null);
  const [ok, setOk] = useState(true);

  useEffect(() => {
    if (!serviceBase) return;
    let raf = 0;
    let last = 0;
    let pending = false;
    const path = sessionFramePath(droneId);
    const tick = (t: number) => {
      if (!pending && t - last > 80 && imgRef.current) {
        last = t;
        pending = true;
        imgRef.current.src = `${serviceBase.replace(/\/$/, "")}${path}?t=${Math.floor(t)}`;
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
  }, [serviceBase, droneId]);

  return (
    <TileFrame
      id={`camera-${droneId}`}
      title={`Camera · ${droneId}`}
      badge={color && <span className="tile-dot" style={{ background: color }} />}
    >
      <div className="camera-body">
        <img
          ref={imgRef}
          alt={`camera ${droneId}`}
          className="camera-img"
          onLoad={() => setOk(true)}
          onError={() => setOk(false)}
        />
        {!ok && <div className="tile-empty">waiting for frames…</div>}
      </div>
    </TileFrame>
  );
}
