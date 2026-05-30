import { useEffect, useState } from "react";
import { TileGrid } from "./components/TileGrid";
import { ReviewGrid } from "./components/ReviewGrid";
import { SessionPicker } from "./components/SessionPicker";
import { NewSessionModal } from "./components/NewSessionModal";
import { FlightPanel } from "./components/panels/FlightPanel";
import { DronesPanel } from "./components/panels/DronesPanel";
import { BrainPanel } from "./components/panels/BrainPanel";
import { ModelsPanel } from "./components/panels/ModelsPanel";
import { ConnectionsPanel } from "./components/panels/ConnectionsPanel";
import { ConfigPanel } from "./components/panels/ConfigPanel";
import { useSession, type RhsTab } from "./store/SessionContext";
import { api } from "./api/client";
import { BrainIcon, ConfigIcon, ConnectionsIcon, DroneIcon, FlightIcon, ModelIcon, MoonIcon, PanelIcon, PauseIcon, PlayIcon, PlusIcon, SunIcon } from "./components/icons";
import type { ComponentType, SVGProps } from "react";

type Theme = "dark" | "light";

function useTheme(): [Theme, () => void] {
  const [theme, setTheme] = useState<Theme>(() => {
    const saved = typeof localStorage !== "undefined" ? localStorage.getItem("theme") : null;
    return saved === "light" || saved === "dark" ? saved : "light";
  });
  useEffect(() => {
    document.documentElement.dataset.theme = theme;
    localStorage.setItem("theme", theme);
  }, [theme]);
  return [theme, () => setTheme((t) => (t === "dark" ? "light" : "dark"))];
}

const TABS: { id: RhsTab; label: string; Icon: ComponentType<SVGProps<SVGSVGElement> & { size?: number }> }[] = [
  { id: "flight", label: "Flight", Icon: FlightIcon },
  { id: "drones", label: "Drones", Icon: DroneIcon },
  { id: "brain", label: "Brain", Icon: BrainIcon },
  { id: "models", label: "Models", Icon: ModelIcon },
  { id: "connections", label: "Connections", Icon: ConnectionsIcon },
  { id: "config", label: "Config", Icon: ConfigIcon },
];

export function App() {
  const {
    health,
    transport,
    state,
    snapshot,
    rhsTab,
    setRhsTab,
    rhsCollapsed,
    setRhsCollapsed,
    reviewSessionId,
    setNewSessionOpen,
  } = useSession();

  const [theme, toggleTheme] = useTheme();

  const session = snapshot?.session;
  const isSimActive = Boolean(session?.active) && session?.kind === "sim";
  const paused = Boolean(session?.paused);
  const togglePause = () => {
    if (paused) void api.sessionResume();
    else void api.sessionPause();
  };

  const reviewed = reviewSessionId
    ? state?.environments.flatMap((e) => e.sessions).find((s) => s.id === reviewSessionId) ?? null
    : null;

  return (
    <div className={`shell${rhsCollapsed ? " rhs-collapsed" : ""}`}>
      <header className="topbar">
        <div className="topbar-left">
          <div className="brand">
            <span className="brand-mark">◆</span>
            <span className="brand-name">Drone Control Station</span>
          </div>
          <SessionPicker />
        </div>
        <div className="topbar-status">
          {isSimActive && (
            <button
              type="button"
              className={`sim-pause-btn${paused ? " is-paused" : ""}`}
              title={paused ? "Resume simulation" : "Pause simulation"}
              aria-label={paused ? "Resume simulation" : "Pause simulation"}
              onClick={togglePause}
            >
              {paused ? <PlayIcon size={15} /> : <PauseIcon size={15} />}
              <span>{paused ? "Resume" : "Pause"}</span>
            </button>
          )}
          <button type="button" className="new-session-btn" onClick={() => setNewSessionOpen(true)}>
            <PlusIcon size={15} />
            <span>New session</span>
          </button>
          <span className={`status-dot status-${health}`} />
          <span className="status-text">
            {health === "ready" ? "ready" : health} · {transport}
          </span>
          <button
            type="button"
            className="theme-toggle"
            title={theme === "dark" ? "Switch to light" : "Switch to dark"}
            aria-label={theme === "dark" ? "Switch to light theme" : "Switch to dark theme"}
            onClick={toggleTheme}
          >
            {theme === "dark" ? <SunIcon size={16} /> : <MoonIcon size={16} />}
          </button>
          <button
            type="button"
            className={`rhs-toggle${rhsCollapsed ? " is-collapsed" : ""}`}
            title={rhsCollapsed ? "Show panel" : "Hide panel"}
            aria-label={rhsCollapsed ? "Show panel" : "Hide panel"}
            onClick={() => setRhsCollapsed(!rhsCollapsed)}
          >
            <PanelIcon size={16} />
          </button>
        </div>
      </header>

      <div className="workspace">
        <main className="wall">
          {reviewed ? <ReviewGrid session={reviewed} /> : <TileGrid />}
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
                  <tab.Icon size={15} />
                  <span>{tab.label}</span>
                </button>
              ))}
            </nav>
            <div className="rhs-body">
              {rhsTab === "flight" && <FlightPanel />}
              {rhsTab === "drones" && <DronesPanel />}
              {rhsTab === "brain" && <BrainPanel />}
              {rhsTab === "models" && <ModelsPanel />}
              {rhsTab === "connections" && <ConnectionsPanel />}
              {rhsTab === "config" && <ConfigPanel />}
            </div>
          </aside>
        )}
      </div>

      <NewSessionModal />
    </div>
  );
}
