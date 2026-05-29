import { useSession } from "../store/SessionContext";
import { CameraTile } from "./tiles/CameraTile";
import { SegOverlayTile } from "./tiles/SegOverlayTile";
import { TrajectoryTile } from "./tiles/TrajectoryTile";
import { WorldSegTile } from "./tiles/WorldSegTile";
import { SplatTile } from "./tiles/SplatTile";

/**
 * The main video-tile wall. One camera + segmentation tile per drone, plus the
 * shared 3D tiles (trajectories, world objects, splat). Each tile maximizes
 * independently via its header button (CSS-driven fullscreen overlay).
 */
export function TileGrid() {
  const { snapshot } = useSession();
  const session = snapshot?.session;
  const drones = session?.drones ?? [];
  const tracks = session?.trajectories ?? [];
  const colorOf = (id: string) => tracks.find((t) => t.droneId === id)?.color;

  if (!session?.active) {
    return (
      <div className="wall-empty">
        <div className="wall-empty-card">
          <h2>No active flight session</h2>
          <p>
            Start a session from the <strong>Flight</strong> tab — choose a simulated or real
            environment and the drones will begin streaming here.
          </p>
        </div>
      </div>
    );
  }

  return (
    <div className="tile-grid">
      {drones.map((id) => (
        <CameraTile key={`cam-${id}`} droneId={id} color={colorOf(id)} />
      ))}
      <TrajectoryTile />
      <SplatTile />
      {drones.map((id) => (
        <SegOverlayTile key={`seg-${id}`} droneId={id} />
      ))}
      <WorldSegTile />
    </div>
  );
}
