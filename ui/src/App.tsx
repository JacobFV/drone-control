import { useEffect, useState } from "react";
import { TileGrid } from "./components/TileGrid";
import { ReviewGrid } from "./components/ReviewGrid";
import { SessionPicker } from "./components/SessionPicker";
import { NewSessionModal } from "./components/NewSessionModal";
import { FlightPanel } from "./components/panels/FlightPanel";
import { ConnectionsPanel } from "./components/panels/ConnectionsPanel";
import { ConfigPanel } from "./components/panels/ConfigPanel";
import { useSession, type RhsTab } from "./store/SessionContext";
import { ConfigIcon, ConnectionsIcon, FlightIcon, MoonIcon, PanelIcon, PlusIcon, SunIcon } from "./components/icons";
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
  { id: "connections", label: "Connections", Icon: ConnectionsIcon },
  { id: "config", label: "Config", Icon: ConfigIcon },
];

export function App() {
  const {
    health,
    transport,
    state,
    rhsTab,
    setRhsTab,
    rhsCollapsed,
    setRhsCollapsed,
    reviewSessionId,
    setNewSessionOpen,
  } = useSession();

  const [theme, toggleTheme] = useTheme();

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
