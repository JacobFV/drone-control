import { useEffect, useRef, useState } from "react";
import { useSession } from "../store/SessionContext";
import { api } from "../api/client";
import { TileFrame } from "./tiles/TileFrame";
import { TrajectoryTile } from "./tiles/TrajectoryTile";
import { WorldSegTile } from "./tiles/WorldSegTile";
import type { RecordEntry, Session, TrajectoryDrone, WorldObject } from "../api/types";

const PALETTE = ["#7fd1ff", "#ffd35a", "#8be0a0", "#f0a39d", "#c9a3ff", "#ff9f5a", "#5ad8d8", "#e069c8"];

/** Read-only rendering of a stored (stopped) session, built from its records. */
export function ReviewGrid({ session }: { session: Session }) {
  const [tracks, setTracks] = useState<TrajectoryDrone[]>([]);
  const [objects, setObjects] = useState<WorldObject[]>([]);

  const cameraRecords = session.records.filter((r) => r.source === "camera" && r.streamUrl);
  const poseRecords = session.records.filter((r) => r.source === "pose");
  const segWorld = [...session.records].reverse().find((r) => r.source === "seg-world");
  const splatRecord = [...session.records].reverse().find((r) => r.type === "gaussian-splat");

  useEffect(() => {
    let cancelled = false;
    (async () => {
      const built: TrajectoryDrone[] = [];
      for (let i = 0; i < poseRecords.length; i += 1) {
        const rec = poseRecords[i];
        const res = await api.getRecordPoseTrack(rec.id);
        if (res?.poses?.length) {
          built.push({
            droneId: rec.droneId ?? rec.id,
            color: PALETTE[i % PALETTE.length],
            goal: null,
            poses: res.poses,
          });
        }
      }
      if (!cancelled) setTracks(built);
      if (segWorld) {
        const data = await api.getRecordArtifact<WorldObject[]>(segWorld.id);
        if (!cancelled && Array.isArray(data)) setObjects(data);
      } else if (!cancelled) {
        setObjects([]);
      }
    })();
    return () => {
      cancelled = true;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [session.id]);

  return (
    <div className="tile-grid">
      {cameraRecords.map((rec) => (
        <ReviewCameraTile key={rec.id} record={rec} />
      ))}
      <TrajectoryTile tracks={tracks} />
      {splatRecord ? <ReviewSplatTile recordId={splatRecord.id} /> : null}
      <WorldSegTile objects={objects} />
    </div>
  );
}

function ReviewCameraTile({ record }: { record: RecordEntry }) {
  const { serviceBase } = useSession();
  const ref = useRef<HTMLImageElement>(null);
  useEffect(() => {
    if (serviceBase && record.streamUrl && ref.current) {
      ref.current.src = `${serviceBase.replace(/\/$/, "")}${record.streamUrl}`;
    }
  }, [serviceBase, record.streamUrl]);
  return (
    <TileFrame id={`review-cam-${record.id}`} title={`Recorded · ${record.droneId ?? record.label}`}>
      <div className="camera-body">
        <img ref={ref} alt={record.label} className="camera-img" />
      </div>
    </TileFrame>
  );
}

function ReviewSplatTile({ recordId }: { recordId: string }) {
  const { serviceBase } = useSession();
  const url = serviceBase ? `${serviceBase.replace(/\/$/, "")}/api/records/${recordId}/splat-viewer` : null;
  return (
    <TileFrame id={`review-splat-${recordId}`} title="Gaussian splat (3D)" interactive={Boolean(url)}>
      {url ? <iframe className="splat-frame" src={url} title="splat" /> : <div className="tile-empty">loading…</div>}
    </TileFrame>
  );
}
