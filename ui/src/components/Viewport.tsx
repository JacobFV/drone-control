import { useEffect, useState } from "react";
import { useStation } from "../store/StationContext";
import { absoluteServiceUrl } from "../api/client";
import { Sim3D } from "./Sim3D";
import { WorldModelView } from "./WorldModelView";
import type { RecordEntry } from "../api/types";

function useAbsolute(path: string | undefined): string {
  const [url, setUrl] = useState("");
  useEffect(() => {
    let active = true;
    if (!path) {
      setUrl("");
      return;
    }
    void absoluteServiceUrl(path).then((u) => active && setUrl(u));
    return () => {
      active = false;
    };
  }, [path]);
  return url;
}

function firstStreamRecord(records: RecordEntry[] | undefined): RecordEntry | undefined {
  return records?.find((r) => r.streamUrl);
}

function CameraView({ label }: { label: string }) {
  const { selectedFlight } = useStation();
  const record = firstStreamRecord(selectedFlight?.records);
  const url = useAbsolute(record?.streamUrl);
  return (
    <div className="camera-view">
      {url ? (
        <img className="camera-frame" src={url} alt={`${label} camera`} />
      ) : (
        <div className="camera-empty">
          <p>No {label.toLowerCase()} stream for this flight.</p>
        </div>
      )}
      <div className="camera-strip">
        <span>{label}</span>
        <span className="mono">{record?.type ?? "—"}</span>
        <span className="mono">{record?.id ?? ""}</span>
      </div>
    </div>
  );
}

export function Viewport() {
  const { mainView } = useStation();
  return (
    <main className="viewport">
      {mainView === "forward" && <CameraView label="Forward" />}
      {mainView === "downward" && <CameraView label="Downward" />}
      {mainView === "simulation" && <Sim3D />}
      {mainView === "world" && <WorldModelView />}
    </main>
  );
}
