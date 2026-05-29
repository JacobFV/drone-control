import { Button, Pill, SegmentedControl } from "./primitives";
import { useStation, type DataSource, type MainView } from "../store/StationContext";

const VIEW_OPTIONS: { value: MainView; label: string }[] = [
  { value: "forward", label: "Forward" },
  { value: "downward", label: "Down" },
  { value: "cameras", label: "Cameras" },
  { value: "simulation", label: "Trajectories" },
  { value: "world", label: "World Model" },
];

const SOURCE_OPTIONS: { value: DataSource; label: string }[] = [
  { value: "sim", label: "Sim" },
  { value: "real", label: "Real" },
];

export function TitleBar() {
  const {
    health,
    mainView,
    setMainView,
    dataSource,
    setDataSource,
    lhsCollapsed,
    setLhsCollapsed,
    rhsCollapsed,
    setRhsCollapsed,
    setSettingsOpen,
  } = useStation();

  const sourceAware = mainView === "cameras" || mainView === "simulation";

  const healthTone = health === "ready" ? "ok" : health === "error" ? "danger" : "default";
  const healthLabel = health === "ready" ? "Ready" : health === "error" ? "Service error" : "Starting…";

  return (
    <header className="titlebar">
      <div className="titlebar-group">
        <Button onClick={() => setLhsCollapsed(!lhsCollapsed)} aria-pressed={!lhsCollapsed}>
          Drones
        </Button>
        <span className="titlebar-brand">Drone Control Station</span>
      </div>

      <div className="titlebar-group titlebar-center">
        <SegmentedControl options={VIEW_OPTIONS} value={mainView} onChange={setMainView} ariaLabel="Main view" />
        {sourceAware && (
          <SegmentedControl options={SOURCE_OPTIONS} value={dataSource} onChange={setDataSource} ariaLabel="Data source" />
        )}
      </div>

      <div className="titlebar-group">
        <Pill tone={healthTone}>{healthLabel}</Pill>
        <Button onClick={() => setSettingsOpen(true)}>Settings</Button>
        <Button onClick={() => setRhsCollapsed(!rhsCollapsed)} aria-pressed={!rhsCollapsed}>
          Inspector
        </Button>
      </div>
    </header>
  );
}
