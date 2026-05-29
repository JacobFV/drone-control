import { TileGrid } from "./components/TileGrid";
import { FlightPanel } from "./components/panels/FlightPanel";
import { ConnectionsPanel } from "./components/panels/ConnectionsPanel";
import { ConfigPanel } from "./components/panels/ConfigPanel";
import { useSession, type RhsTab } from "./store/SessionContext";

const TABS: { id: RhsTab; label: string }[] = [
  { id: "flight", label: "Flight" },
  { id: "connections", label: "Connections" },
  { id: "config", label: "Config" },
];

export function App() {
  const { health, transport, snapshot, rhsTab, setRhsTab, rhsCollapsed, setRhsCollapsed } = useSession();
  const active = snapshot?.session.active;

  return (
    <div className={`shell${rhsCollapsed ? " rhs-collapsed" : ""}`}>
      <header className="topbar">
        <div className="brand">
          <span className="brand-mark">◆</span>
          <span className="brand-name">Drone Control Station</span>
        </div>
        <div className="topbar-status">
          <span className={`status-dot status-${health}`} />
          <span className="status-text">
            {health === "ready" ? "service ready" : health}
            {" · "}
            {transport}
            {active ? " · live session" : " · idle"}
          </span>
          <button type="button" className="rhs-toggle" onClick={() => setRhsCollapsed(!rhsCollapsed)}>
            {rhsCollapsed ? "◀ panel" : "panel ▶"}
          </button>
        </div>
      </header>

      <div className="workspace">
        <main className="wall">
          <TileGrid />
        </main>

        {!rhsCollapsed && (
          <aside className="rhs">
            <nav className="rhs-tabs">
              {TABS.map((tab) => (
                <button
                  key={tab.id}
                  type="button"
                  className={`rhs-tab${rhsTab === tab.id ? " is-active" : ""}`}
                  onClick={() => setRhsTab(tab.id)}
                >
                  {tab.label}
                </button>
              ))}
            </nav>
            <div className="rhs-body">
              {rhsTab === "flight" && <FlightPanel />}
              {rhsTab === "connections" && <ConnectionsPanel />}
              {rhsTab === "config" && <ConfigPanel />}
            </div>
          </aside>
        )}
      </div>
    </div>
  );
}
