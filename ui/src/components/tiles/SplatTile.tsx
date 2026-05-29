import { useMemo } from "react";
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

  const viewerUrl =
    splatRecord && serviceBase
      ? `${serviceBase.replace(/\/$/, "")}/api/records/${splatRecord.id}/splat-viewer`
      : null;

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
