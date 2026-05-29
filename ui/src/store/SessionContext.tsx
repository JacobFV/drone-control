import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
  type ReactNode,
} from "react";
import { api, getServiceUrl, onHealthChange, type ServiceHealth } from "../api/client";
import { useLiveSnapshot } from "../api/socket";
import { usePolling } from "../lib/usePolling";
import type { ConfigStatus, NetworkSummary, StationState, WsSnapshot } from "../api/types";

export type RhsTab = "flight" | "connections" | "config";

interface SessionContextValue {
  health: ServiceHealth;
  serviceBase: string;
  transport: "ws" | "poll" | "connecting";

  // live
  snapshot: WsSnapshot | null;

  // persisted state
  state: StationState | null;
  config: ConfigStatus | null;
  network: NetworkSummary | null;

  // UI shell
  rhsTab: RhsTab;
  setRhsTab: (tab: RhsTab) => void;
  rhsCollapsed: boolean;
  setRhsCollapsed: (v: boolean) => void;
  maximizedTile: string | null;
  setMaximizedTile: (id: string | null) => void;

  // realtime commands + refreshers
  send: (command: Record<string, unknown>) => void;
  refreshState: () => Promise<void>;
  refreshConfig: () => Promise<void>;
  refreshNetwork: () => Promise<void>;
}

const SessionContext = createContext<SessionContextValue | null>(null);

export function useSession(): SessionContextValue {
  const ctx = useContext(SessionContext);
  if (!ctx) throw new Error("useSession must be used within SessionProvider");
  return ctx;
}

export function SessionProvider({ children }: { children: ReactNode }) {
  const [health, setHealth] = useState<ServiceHealth>("starting");
  const [serviceBase, setServiceBase] = useState("");
  const [state, setState] = useState<StationState | null>(null);
  const [config, setConfig] = useState<ConfigStatus | null>(null);
  const [network, setNetwork] = useState<NetworkSummary | null>(null);

  const [rhsTab, setRhsTab] = useState<RhsTab>("flight");
  const [rhsCollapsed, setRhsCollapsed] = useState(false);
  const [maximizedTile, setMaximizedTile] = useState<string | null>(null);

  const { snapshot, transport, send } = useLiveSnapshot();

  useEffect(() => onHealthChange(setHealth), []);
  useEffect(() => {
    void getServiceUrl().then(setServiceBase);
  }, []);

  const refreshState = useCallback(async () => {
    const result = await api.getState();
    if (result) setState(result);
  }, []);
  const refreshConfig = useCallback(async () => {
    const result = await api.getConfig();
    if (result) setConfig(result);
  }, []);
  const refreshNetwork = useCallback(async () => {
    const result = await api.getNetwork();
    if (result) setNetwork(result);
  }, []);

  useEffect(() => {
    void refreshState();
    void refreshConfig();
    void refreshNetwork();
  }, [refreshState, refreshConfig, refreshNetwork]);

  // Persisted state (environments/sessions/records) refreshes slowly; the live
  // session telemetry comes from the WS/poll snapshot instead.
  usePolling(refreshState, 4000);

  const value = useMemo<SessionContextValue>(
    () => ({
      health,
      serviceBase,
      transport,
      snapshot,
      state,
      config,
      network,
      rhsTab,
      setRhsTab,
      rhsCollapsed,
      setRhsCollapsed,
      maximizedTile,
      setMaximizedTile,
      send,
      refreshState,
      refreshConfig,
      refreshNetwork,
    }),
    [
      health, serviceBase, transport, snapshot, state, config, network,
      rhsTab, rhsCollapsed, maximizedTile, send, refreshState, refreshConfig, refreshNetwork,
    ],
  );

  return <SessionContext.Provider value={value}>{children}</SessionContext.Provider>;
}
