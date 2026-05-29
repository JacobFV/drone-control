import type { ReactNode } from "react";
import { useSession } from "../../store/SessionContext";
import { MaximizeIcon, RestoreIcon } from "../icons";

interface TileFrameProps {
  id: string;
  title: string;
  badge?: ReactNode;
  interactive?: boolean; // shows a "drag to orbit" hint
  children: ReactNode;
}

/** A single wall tile: header (title + badge + maximize) and a body. */
export function TileFrame({ id, title, badge, interactive, children }: TileFrameProps) {
  const { maximizedTile, setMaximizedTile } = useSession();
  const isMax = maximizedTile === id;
  return (
    <div className={`tile${isMax ? " is-max" : ""}`}>
      <div className="tile-header">
        <span className="tile-title">{title}</span>
        <div className="tile-actions">
          {interactive && <span className="tile-hint">drag · scroll</span>}
          {badge}
          <button
            type="button"
            className="tile-max"
            title={isMax ? "Restore" : "Maximize"}
            aria-label={isMax ? "Restore" : "Maximize"}
            onClick={() => setMaximizedTile(isMax ? null : id)}
          >
            {isMax ? <RestoreIcon size={14} /> : <MaximizeIcon size={14} />}
          </button>
        </div>
      </div>
      <div className="tile-body">{children}</div>
    </div>
  );
}
