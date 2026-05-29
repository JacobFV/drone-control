import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from "react";
import { api, onHealthChange, type ServiceHealth } from "../api/client";
import { usePolling } from "../lib/usePolling";
import type {
  ConfigStatus,
  Drone,
  Flight,
  ManualStatus,
  NetworkSummary,
  ReconstructionStatus,
  RuntimeDrone,
  RuntimeStatus,
  SessionStatus,
  StationState,
} from "../api/types";

export type WorkflowStep = "connect" | "fly" | "record" | "reconstruct";
export type MainView = "forward" | "downward" | "simulation" | "world";

interface StationContextValue {
  // service
  health: ServiceHealth;

  // domain state
  drones: Drone[];
  selectedDroneId: string;
  selectedFlightId: string;
  selectedRecordId: string;
  selectedDrone: Drone | undefined;
  selectedFlight: Flight | undefined;
  selectedRuntimeDrone: RuntimeDrone | undefined;

  config: ConfigStatus | null;
  network: NetworkSummary | null;
  manualStatus: ManualStatus | null;
  sessionStatus: SessionStatus | null;
  reconstructionStatus: ReconstructionStatus | null;
  runtimeStatus: RuntimeStatus | null;

  // UI shell
  step: WorkflowStep;
  mainView: MainView;
  settingsOpen: boolean;
  lhsCollapsed: boolean;
  rhsCollapsed: boolean;

  // setters
  setStep: (step: WorkflowStep) => void;
  setMainView: (view: MainView) => void;
  setSettingsOpen: (open: boolean) => void;
  setLhsCollapsed: (v: boolean) => void;
  setRhsCollapsed: (v: boolean) => void;
  setSelectedRecordId: (id: string) => void;
  selectDrone: (id: string) => void;
  selectFlight: (droneId: string, flightId: string) => void;

  // refresh / mutations
  refreshState: () => Promise<void>;
  refreshManual: () => Promise<void>;
  refreshSession: () => Promise<void>;
  refreshReconstruction: () => Promise<void>;
  refreshRuntime: () => Promise<void>;
  refreshNetwork: () => Promise<void>;
  refreshConfig: () => Promise<void>;
  setManualStatus: (status: ManualStatus | null) => void;
  setRuntimeStatus: (status: RuntimeStatus | null) => void;
  setSessionStatus: (status: SessionStatus | null) => void;
  setReconstructionStatus: (status: ReconstructionStatus | null) => void;
  setConfig: (config: ConfigStatus | null) => void;
  setNetwork: (network: NetworkSummary | null) => void;
}

const StationContext = createContext<StationContextValue | null>(null);

export function useStation(): StationContextValue {
  const ctx = useContext(StationContext);
  if (!ctx) throw new Error("useStation must be used within StationProvider");
  return ctx;
}

export function StationProvider({ children }: { children: ReactNode }) {
  const [health, setHealthState] = useState<ServiceHealth>("starting");

  const [drones, setDrones] = useState<Drone[]>([]);
  const [selectedDroneId, setSelectedDroneId] = useState("");
  const [selectedFlightId, setSelectedFlightId] = useState("");
  const [selectedRecordId, setSelectedRecordId] = useState("");

  const [config, setConfig] = useState<ConfigStatus | null>(null);
  const [network, setNetwork] = useState<NetworkSummary | null>(null);
  const [manualStatus, setManualStatus] = useState<ManualStatus | null>(null);
  const [sessionStatus, setSessionStatus] = useState<SessionStatus | null>(null);
  const [reconstructionStatus, setReconstructionStatus] =
    useState<ReconstructionStatus | null>(null);
  const [runtimeStatus, setRuntimeStatus] = useState<RuntimeStatus | null>(null);

  const [step, setStep] = useState<WorkflowStep>("connect");
  const [mainView, setMainView] = useState<MainView>("forward");
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [lhsCollapsed, setLhsCollapsed] = useState(false);
  const [rhsCollapsed, setRhsCollapsed] = useState(false);

  // Track current selection in refs so async refreshers read fresh values.
  const selDroneRef = useRef(selectedDroneId);
  const selFlightRef = useRef(selectedFlightId);
  selDroneRef.current = selectedDroneId;
  selFlightRef.current = selectedFlightId;
  const reconFlightRef = useRef("");

  useEffect(() => onHealthChange(setHealthState), []);

  const selectedDrone = useMemo(
    () => drones.find((d) => d.id === selectedDroneId),
    [drones, selectedDroneId],
  );
  const selectedFlight = useMemo(
    () => selectedDrone?.flights.find((f) => f.id === selectedFlightId),
    [selectedDrone, selectedFlightId],
  );
  const selectedRuntimeDrone = useMemo(
    () => runtimeStatus?.drones?.find((d) => d.droneId === selectedDroneId),
    [runtimeStatus, selectedDroneId],
  );

  const refreshState = useCallback(async () => {
    const result = await api.getState();
    if (!result) return;
    applyState(result);
  }, []);

  const applyState = useCallback((result: StationState) => {
    setDrones(result.drones);
    setSelectedDroneId((prev) => {
      const next = result.drones.find((d) => d.id === prev)?.id ?? result.drones[0]?.id ?? "";
      return next;
    });
    setSelectedFlightId((prevFlight) => {
      const droneId = selDroneRef.current;
      const drone = result.drones.find((d) => d.id === droneId) ?? result.drones[0];
      const stillThere = drone?.flights.find((f) => f.id === prevFlight)?.id;
      return stillThere ?? drone?.flights[0]?.id ?? "";
    });
  }, []);

  const refreshManual = useCallback(async () => {
    const status = await api.getManualStatus();
    if (status) setManualStatus(status);
  }, []);

  const refreshSession = useCallback(async () => {
    const flightId = selFlightRef.current;
    if (!flightId) {
      setSessionStatus(null);
      return;
    }
    const status = await api.getSession(flightId);
    if (status) setSessionStatus(status);
  }, []);

  const refreshReconstruction = useCallback(async () => {
    const flightId = selFlightRef.current;
    if (!flightId) {
      setReconstructionStatus(null);
      reconFlightRef.current = "";
      return;
    }
    if (flightId !== reconFlightRef.current) {
      reconFlightRef.current = flightId;
      setReconstructionStatus(null);
    }
    const wasActive = Boolean(reconstructionStatusRef.current?.job?.active);
    const status = await api.getReconstructionStatus(flightId);
    if (status) setReconstructionStatus(status);
    if (wasActive && status?.job && !status.job.active) {
      await refreshState();
    }
  }, [refreshState]);

  const reconstructionStatusRef = useRef<ReconstructionStatus | null>(null);
  reconstructionStatusRef.current = reconstructionStatus;

  const refreshRuntime = useCallback(async () => {
    const status = await api.getRuntimeStatus();
    if (status) setRuntimeStatus(status);
  }, []);

  const refreshNetwork = useCallback(async () => {
    const result = await api.getNetwork();
    if (result) setNetwork(result);
  }, []);

  const refreshConfig = useCallback(async () => {
    const result = await api.getConfig();
    if (result) setConfig(result);
  }, []);

  const selectDrone = useCallback((id: string) => {
    setSelectedDroneId(id);
    setSelectedFlightId((prev) => prev);
  }, []);

  const selectFlight = useCallback((droneId: string, flightId: string) => {
    setSelectedDroneId(droneId);
    setSelectedFlightId(flightId);
    setSelectedRecordId("");
  }, []);

  // ----- Initial load -----
  useEffect(() => {
    let cancelled = false;
    (async () => {
      const initial = await api.getState();
      if (cancelled) return;
      if (initial) {
        setDrones(initial.drones);
        setSelectedDroneId(initial.drones[0]?.id ?? "");
        setSelectedFlightId(initial.drones[0]?.flights[0]?.id ?? "");
      }
      await Promise.all([refreshConfig(), refreshNetwork(), refreshManual()]);
    })();
    return () => {
      cancelled = true;
    };
  }, [refreshConfig, refreshNetwork, refreshManual]);

  // ----- Background polling (matches original cadences) -----
  usePolling(refreshState, 5000);
  usePolling(refreshSession, 5000);
  usePolling(refreshReconstruction, 2000);
  usePolling(refreshRuntime, 1000);

  // When the selected flight changes, refresh its session + reconstruction.
  useEffect(() => {
    void refreshSession();
    void refreshReconstruction();
  }, [selectedFlightId, refreshSession, refreshReconstruction]);

  const value: StationContextValue = {
    health,
    drones,
    selectedDroneId,
    selectedFlightId,
    selectedRecordId,
    selectedDrone,
    selectedFlight,
    selectedRuntimeDrone,
    config,
    network,
    manualStatus,
    sessionStatus,
    reconstructionStatus,
    runtimeStatus,
    step,
    mainView,
    settingsOpen,
    lhsCollapsed,
    rhsCollapsed,
    setStep,
    setMainView,
    setSettingsOpen,
    setLhsCollapsed,
    setRhsCollapsed,
    setSelectedRecordId,
    selectDrone,
    selectFlight,
    refreshState,
    refreshManual,
    refreshSession,
    refreshReconstruction,
    refreshRuntime,
    refreshNetwork,
    refreshConfig,
    setManualStatus,
    setRuntimeStatus,
    setSessionStatus,
    setReconstructionStatus,
    setConfig,
    setNetwork,
  };

  return <StationContext.Provider value={value}>{children}</StationContext.Provider>;
}
