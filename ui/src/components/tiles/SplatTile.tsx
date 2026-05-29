import { useEffect, useMemo, useState } from "react";
import { useSession } from "../../store/SessionContext";
import { openSplatViewer } from "../../api/client";
import { TileFrame } from "./TileFrame";

/**
 * Gaussian-splat tile. For a stored splat record we embed the backend's gsplat
 * orbit viewer (interactive). For a live session we surface world-model status
 * (the live splat is built by the runtime for real environments).
 */
export function SplatTile() {
  const { snapshot, state, serviceBase } = useSession();
  const sessionId = snapshot?.session.sessionId;
  const world = snapshot?.session.worldModel;

  const splatRecord = useMemo(() => {
    if (!state || !sessionId) return null;
    for (const env of state.environments) {
      for (const session of env.sessions) {
        if (session.id !== sessionId) continue;
        const rec = [...session.records].reverse().find((r) => r.type === "gaussian-splat");
        if (rec) return rec;
      }
    }
    return null;
  }, [state, sessionId]);

  const base = serviceBase ? serviceBase.replace(/\/$/, "") : null;
  const isLive = Boolean(base && world?.running && (world.gaussians ?? 0) > 0);

  // The splat keeps converging (gaussians densify, loss drops). The gsplat viewer
  // loads its .ply once, so periodically reload the live viewer to show progress.
  const [tick, setTick] = useState(0);
  useEffect(() => {
    if (!isLive) return;
    const id = window.setInterval(() => setTick((t) => t + 1), 6000);
    return () => window.clearInterval(id);
  }, [isLive]);

  const liveUrl = isLive ? `${base}/api/session/splat/viewer?r=${tick}` : null;
  const viewerUrl = liveUrl ?? (splatRecord && base ? `${base}/api/records/${splatRecord.id}/splat-viewer` : null);

  return (
    <TileFrame
      id="splat"
      title="Gaussian splat (3D)"
      interactive={Boolean(viewerUrl)}
      badge={
        world?.running ? (
          <span className="tile-count">{world.gaussians ?? 0}</span>
        ) : undefined
      }
    >
      {viewerUrl ? (
        <iframe className="splat-frame" src={viewerUrl} title="gaussian splat viewer" />
      ) : (
        <div className="splat-status">
          {world?.running ? (
            <>
              <div className="splat-line">building live splat…</div>
              <div className="splat-sub">{world.gaussians ?? 0} gaussians</div>
            </>
          ) : (
            <>
              <div className="splat-line">no splat yet</div>
              <div className="splat-sub">{world?.reason ?? "start a real session with the world model"}</div>
            </>
          )}
          {splatRecord && (
            <button type="button" className="tile-mini" onClick={() => openSplatViewer(splatRecord.id)}>
              open stored splat
            </button>
          )}
        </div>
      )}
    </TileFrame>
  );
}
