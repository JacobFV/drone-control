import { TitleBar } from "./components/TitleBar";
import { DroneTree } from "./components/DroneTree";
import { Viewport } from "./components/Viewport";
import { Inspector } from "./components/Inspector";
import { SettingsDrawer } from "./components/SettingsDrawer";
import { useStation } from "./store/StationContext";

export function App() {
  const { lhsCollapsed, rhsCollapsed, settingsOpen } = useStation();
  const shellClass = [
    "app-shell",
    lhsCollapsed && "lhs-collapsed",
    rhsCollapsed && "rhs-collapsed",
  ]
    .filter(Boolean)
    .join(" ");

  return (
    <div className={shellClass}>
      <TitleBar />
      <div className="workspace">
        <DroneTree />
        <Viewport />
        <Inspector />
      </div>
      {settingsOpen && <SettingsDrawer />}
    </div>
  );
}
