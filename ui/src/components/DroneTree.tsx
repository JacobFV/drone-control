import { useMemo, useState } from "react";
import { Button, Pill } from "./primitives";
import { useStation } from "../store/StationContext";
import { usePolling } from "../lib/usePolling";
import { api } from "../api/client";
import { upper } from "../lib/format";
import type { Drone, RuntimeDrone, SimStatus } from "../api/types";

interface Unit {
  id: string;
  name: string;
  runtime?: RuntimeDrone;
  stored?: Drone;
  online: boolean;
  armed: boolean;
  controller?: string;
  linkState?: string;
}

export function DroneTree() {
  const {
    drones,
    runtimeStatus,
    selectedDroneId,
    selectedDroneIds,
    selectDrone,
    selectFlight,
    selectedFlightId,
    toggleDroneSelected,
    setSelectedDroneIds,
  } = useStation();
  const [expanded, setExpanded] = useState<Record<string, boolean>>({});
  const [sim, setSim] = useState<SimStatus | null>(null);
  usePolling(async () => {
    const status = await api.getSimStatus();
    if (status) setSim(status);
  }, 1500);

  // Unify the controllable runtime drones with stored drones (by id). Runtime
  // units come first (operable); stored-only drones follow as review archives.
  const units = useMemo<Unit[]>(() => {
    const runtimeDrones = runtimeStatus?.drones ?? [];
    const storedById = new Map(drones.map((d) => [d.id, d]));
    const seen = new Set<string>();
    const list: Unit[] = [];
    for (const rt of runtimeDrones) {
      seen.add(rt.droneId);
      const stored = storedById.get(rt.droneId);
      list.push({
        id: rt.droneId,
        name: stored?.name || rt.droneId,
        runtime: rt,
        stored,
        online: Boolean(rt.running),
        armed: Boolean(rt.safety?.armed),
        controller: rt.controller,
        linkState: rt.linkState,
      });
    }
    for (const d of drones) {
      if (seen.has(d.id)) continue;
      list.push({ id: d.id, name: d.name || d.id, stored: d, online: false, armed: false });
    }
    return list;
  }, [runtimeStatus, drones]);

  const allIds = units.map((u) => u.id);
  const allSelected = allIds.length > 0 && allIds.every((id) => selectedDroneIds.includes(id));
  const toggle = (id: string) => setExpanded((p) => ({ ...p, [id]: !p[id] }));

  return (
    <aside className="sidebar lhs">
      <div className="sidebar-header">
        <span className="section-label">Drones</span>
        <span className="count-badge">{selectedDroneIds.length}/{units.length}</span>
      </div>
      <div className="select-bar">
        <Button onClick={() => setSelectedDroneIds(allSelected ? [] : allIds)} disabled={!units.length}>
          {allSelected ? "Select none" : "Select all"}
        </Button>
        {selectedDroneIds.length > 0 && (
          <span className="select-count">{selectedDroneIds.length} selected</span>
        )}
      </div>

      <div className="tree">
        {units.length === 0 && <p className="empty">No drones. Discover, or start a sim.</p>}
        {units.map((unit) => {
          const isPrimary = unit.id === selectedDroneId;
          const inSet = selectedDroneIds.includes(unit.id);
          const isOpen = expanded[unit.id] ?? false;
          const flights = unit.stored?.flights ?? [];
          return (
            <div key={unit.id} className={`tree-node${inSet ? " is-selected" : ""}`}>
              <div className={`tree-row drone-row${isPrimary ? " is-primary" : ""}`}>
                <input
                  type="checkbox"
                  className="unit-check"
                  checked={inSet}
                  onChange={() => toggleDroneSelected(unit.id)}
                  onClick={(e) => e.stopPropagation()}
                  aria-label={`select ${unit.name}`}
                />
                <button type="button" className="unit-main" onClick={() => selectDrone(unit.id)}>
                  <span className={`status-dot${unit.online ? " is-online" : ""}${unit.armed ? " is-armed" : ""}`} />
                  <span className="tree-name">{unit.name}</span>
                  <span className="tree-sub">{unit.runtime ? upper(unit.controller || "—") : "archive"}</span>
                </button>
                {flights.length > 0 && (
                  <button type="button" className={`chevron${isOpen ? " is-open" : ""}`} onClick={() => toggle(unit.id)}>
                    ›
                  </button>
                )}
              </div>
              {unit.runtime && (
                <div className="unit-status">
                  <span>{unit.linkState}</span>
                  <span>{unit.armed ? "armed" : "disarmed"}</span>
                  <span className="mono">sent {unit.runtime.sent ?? 0}</span>
                </div>
              )}
              {isOpen && (
                <div className="tree-children">
                  {flights.map((flight) => (
                    <button
                      key={flight.id}
                      type="button"
                      className={`tree-row flight-row${flight.id === selectedFlightId ? " is-active" : ""}`}
                      onClick={() => selectFlight(unit.id, flight.id)}
                    >
                      <span className="tree-name">{flight.name || flight.id}</span>
                      <span className="tree-sub">{upper(flight.mode || "")}</span>
                    </button>
                  ))}
                </div>
              )}
            </div>
          );
        })}
      </div>

      {(sim?.drones?.length ?? 0) > 0 && (
        <>
          <div className="sidebar-header sub">
            <span className="section-label">Sim drones</span>
            <span className="count-badge">{sim?.numDrones}</span>
          </div>
          <div className="tree">
            {(sim?.drones ?? []).map((d) => (
              <div key={d.droneId} className="tree-row drone-row sim-row" title="autonomously driven in the sim">
                <span className="legend-dot" style={{ background: d.color }} />
                <span className="tree-name">{d.droneId}</span>
                <span className="tree-sub mono">{d.distance.toFixed(1)} m</span>
              </div>
            ))}
          </div>
        </>
      )}

      <div className="sidebar-footer">
        <Pill>One drone connection per Wi-Fi radio</Pill>
      </div>
    </aside>
  );
}
