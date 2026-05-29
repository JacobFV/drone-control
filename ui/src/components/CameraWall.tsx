import { useEffect, useState } from "react";
import { useStation } from "../store/StationContext";
import { api, getServiceUrl, runtimeFramePath, simFramePath } from "../api/client";

interface Tile {
  key: string;
  label: string;
  color: string;
  path: string;
}

export function CameraWall() {
  const { dataSource, runtimeStatus } = useStation();
  const [base, setBase] = useState("");
  const [tiles, setTiles] = useState<Tile[]>([]);
  const [tick, setTick] = useState(0);

  useEffect(() => {
    let active = true;
    void getServiceUrl().then((u) => active && setBase(u));
    return () => {
      active = false;
    };
  }, []);

  // Which drones to show (and their per-tile frame endpoint).
  useEffect(() => {
    let cancelled = false;
    const tick = async () => {
      if (dataSource === "sim") {
        const status = await api.getSimStatus();
        if (cancelled) return;
        const drones = status?.drones ?? [];
        setTiles(
          drones.map((d, i) => ({ key: d.droneId, label: d.droneId, color: d.color, path: simFramePath(i) })),
        );
        return;
      }
      const drones = runtimeStatus?.drones ?? [];
      const palette = ["#7fd1ff", "#ffd35a", "#8be0a0", "#f0a39d", "#c9a3ff", "#ff9f5a"];
      if (cancelled) return;
      setTiles(
        drones.map((d, i) => ({
          key: d.droneId,
          label: d.droneId,
          color: palette[i % palette.length],
          path: runtimeFramePath(d.droneId),
        })),
      );
    };
    void tick();
    const id = window.setInterval(tick, 1500);
    return () => {
      cancelled = true;
      window.clearInterval(id);
    };
  }, [dataSource, runtimeStatus]);

  // Frame refresh cadence (~7 fps) via cache-busting query param.
  useEffect(() => {
    const id = window.setInterval(() => setTick((t) => (t + 1) % 100000), 140);
    return () => window.clearInterval(id);
  }, []);

  return (
    <div className="camera-wall">
      {tiles.length === 0 && (
        <div className="camera-empty">
          <p>
            No {dataSource === "sim" ? "sim" : "live"} cameras.{" "}
            {dataSource === "sim" ? "Start a sim session (Swarm · Sim panel)." : "Attach a drone camera."}
          </p>
        </div>
      )}
      {tiles.map((tile) => (
        <figure className="camera-tile" key={tile.key}>
          <img
            className="camera-tile-img"
            src={base ? `${base}${tile.path}?t=${tick}` : ""}
            alt={tile.label}
            onError={(e) => {
              (e.currentTarget as HTMLImageElement).style.visibility = "hidden";
            }}
            onLoad={(e) => {
              (e.currentTarget as HTMLImageElement).style.visibility = "visible";
            }}
          />
          <figcaption className="camera-tile-cap">
            <span className="legend-dot" style={{ background: tile.color }} />
            {tile.label}
          </figcaption>
        </figure>
      ))}
    </div>
  );
}
