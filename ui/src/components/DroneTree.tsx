import { useState } from "react";
import { Pill } from "./primitives";
import { useStation } from "../store/StationContext";
import { upper } from "../lib/format";

export function DroneTree() {
  const { drones, selectedDroneId, selectedFlightId, selectDrone, selectFlight } = useStation();
  const [expanded, setExpanded] = useState<Record<string, boolean>>({});

  const toggle = (id: string) => setExpanded((prev) => ({ ...prev, [id]: !prev[id] }));

  return (
    <aside className="sidebar lhs">
      <div className="sidebar-header">
        <span className="section-label">Drones</span>
        <span className="count-badge">{drones.length}</span>
      </div>
      <div className="tree">
        {drones.length === 0 && <p className="empty">No drones discovered yet.</p>}
        {drones.map((drone) => {
          const isOpen = expanded[drone.id] ?? drone.id === selectedDroneId;
          const online = drone.status?.toLowerCase() === "online" || drone.status?.toLowerCase() === "connected";
          return (
            <div key={drone.id} className={`tree-node${drone.id === selectedDroneId ? " is-selected" : ""}`}>
              <button
                type="button"
                className="tree-row drone-row"
                onClick={() => {
                  selectDrone(drone.id);
                  toggle(drone.id);
                }}
              >
                <span className={`status-dot${online ? " is-online" : ""}`} />
                <span className="tree-name">{drone.name || drone.id}</span>
                <span className="tree-sub">{drone.model || ""}</span>
                <span className={`chevron${isOpen ? " is-open" : ""}`}>›</span>
              </button>
              {isOpen && (
                <div className="tree-children">
                  {(drone.flights ?? []).map((flight) => (
                    <button
                      key={flight.id}
                      type="button"
                      className={`tree-row flight-row${flight.id === selectedFlightId ? " is-active" : ""}`}
                      onClick={() => selectFlight(drone.id, flight.id)}
                    >
                      <span className="tree-name">{flight.name || flight.id}</span>
                      <span className="tree-sub">{upper(flight.mode || "")}</span>
                    </button>
                  ))}
                  {(drone.flights ?? []).length === 0 && <p className="empty small">No flights</p>}
                </div>
              )}
            </div>
          );
        })}
      </div>
      <div className="sidebar-footer">
        <Pill>One drone connection per Wi-Fi radio</Pill>
      </div>
    </aside>
  );
}
