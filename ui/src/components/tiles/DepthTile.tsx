import { useEffect, useRef, useState } from "react";
import { useSession } from "../../store/SessionContext";
import { sessionDepthPath } from "../../api/client";
import { TileFrame } from "./TileFrame";

/** Colorized monocular-depth map for a drone (Depth Anything V2). */
export function DepthTile({ droneId }: { droneId: string }) {
  const { serviceBase, snapshot } = useSession();
  const imgRef = useRef<HTMLImageElement>(null);
  const [ok, setOk] = useState(false);
  const available = snapshot?.session.depth?.available;

  useEffect(() => {
    if (!serviceBase || !available) return;
    let raf = 0;
    let last = 0;
    let pending = false;
    const path = sessionDepthPath(droneId);
    const tick = (t: number) => {
      if (!pending && t - last > 180 && imgRef.current) {
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
  }, [serviceBase, droneId, available]);

  return (
    <TileFrame id={`depth-${droneId}`} title={`Depth · ${droneId}`}>
      <div className="camera-body">
        <img ref={imgRef} alt={`depth ${droneId}`} className="camera-img" onLoad={() => setOk(true)} onError={() => setOk(false)} />
        {!available && <div className="tile-empty">depth model not installed</div>}
        {available && !ok && <div className="tile-empty">estimating depth…</div>}
      </div>
    </TileFrame>
  );
}
