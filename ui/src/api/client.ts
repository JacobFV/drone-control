import { bridge } from "./bridge";
import type {
  ConfigStatus,
  DiscoverResult,
  ManualStatus,
  NetworkSummary,
  SessionStatus,
  StationState,
} from "./types";

let serviceUrlCache = "";
let wsUrlCache: string | null = null;

export type ServiceHealth = "starting" | "ready" | "error";

type HealthListener = (health: ServiceHealth) => void;
const healthListeners = new Set<HealthListener>();

export function onHealthChange(listener: HealthListener): () => void {
  healthListeners.add(listener);
  return () => healthListeners.delete(listener);
}
function setHealth(health: ServiceHealth): void {
  for (const listener of healthListeners) listener(health);
}

export async function getServiceUrl(): Promise<string> {
  if (!serviceUrlCache) serviceUrlCache = await bridge.serviceUrl();
  return serviceUrlCache;
}

export async function getWsUrl(): Promise<string> {
  if (wsUrlCache === null) {
    wsUrlCache = bridge.wsUrl ? await bridge.wsUrl() : "";
  }
  return wsUrlCache;
}

export async function absoluteServiceUrl(path: string): Promise<string> {
  const base = await getServiceUrl();
  return new URL(path, base).toString();
}

export function openExternal(url: string): void {
  if (bridge.openExternal) void bridge.openExternal(url);
  else window.open(url, "_blank", "noopener");
}

export async function request<T = unknown>(
  method: string,
  path: string,
  body?: unknown,
): Promise<T | null> {
  try {
    const result = (await bridge.request({ method, path, body })) as T;
    setHealth("ready");
    return result;
  } catch (error) {
    console.error(`[droneStation] ${method} ${path}`, error);
    setHealth("error");
    return null;
  }
}

export const api = {
  // ----- State / config -----
  getState: () => request<StationState>("GET", "/api/state"),
  getConfig: () => request<ConfigStatus>("GET", "/api/config"),
  getNetwork: () => request<NetworkSummary>("GET", "/api/system/network"),

  // ----- Environments -----
  createEnvironment: (name: string, kind: string, config: Record<string, unknown> = {}) =>
    request<{ id: string }>("POST", "/api/environments", { name, kind, config }),

  // ----- Session (the single active flight session) -----
  getSessionStatus: () => request<SessionStatus>("GET", "/api/session/status"),
  sessionStart: (kind: string, name: string, options: Record<string, unknown>) =>
    request<SessionStatus>("POST", "/api/session/start", { kind, name, options }),
  sessionStop: () => request<SessionStatus>("POST", "/api/session/stop", {}),
  sessionSpeed: (mode: "realtime" | "max") =>
    request<SessionStatus>("POST", "/api/session/speed", { mode }),

  // ----- Pose / reconstruction (per session) -----
  computePoseTrack: (sessionId: string) =>
    request("POST", `/api/sessions/${sessionId}/pose/compute`, {}),
  getReconstructionStatus: (sessionId: string) =>
    request("GET", `/api/sessions/${sessionId}/reconstruction/status`),
  reconstructionStart: (sessionId: string, body: Record<string, unknown> = {}) =>
    request("POST", `/api/sessions/${sessionId}/reconstruction/start`, body),
  reconstructionStop: (sessionId: string) =>
    request("POST", `/api/sessions/${sessionId}/reconstruction/stop`, {}),

  // ----- Discovery / wifi -----
  discoverDrones: (iface: string) =>
    request<DiscoverResult>("POST", "/api/drones/discover", { iface, rescan: true }),
  connectWifi: (iface: string, ssid: string, password: string) =>
    request<{ ok?: boolean }>("POST", "/api/wifi/connect", { iface, ssid, password, confirmDisconnect: true }),
  reconnectWifi: (iface: string, password: string) =>
    request<{ ok?: boolean }>("POST", "/api/wifi/reconnect", { iface, password }),
  getWifiInterfaces: () => request("GET", "/api/wifi/interfaces"),

  // ----- Manual control -----
  getManualStatus: () => request<ManualStatus>("GET", "/api/manual/status"),
  manualArm: () => request<ManualStatus>("POST", "/api/manual/arm", {}),
  manualDisarm: () => request<ManualStatus>("POST", "/api/manual/disarm", {}),
  manualStop: () => request<ManualStatus>("POST", "/api/manual/stop", {}),
  manualConfig: (config: Record<string, unknown>) =>
    request<ConfigStatus>("POST", "/api/manual/config", config),

  // ----- Runtime (real swarm) -----
  getRuntimeStatus: () => request("GET", "/api/runtime/status"),
  runtimeSetController: (droneId: string, mode: string) =>
    request("POST", `/api/runtime/drones/${droneId}/controller`, { mode }),
  runtimeArm: (droneId: string) => request("POST", `/api/runtime/drones/${droneId}/arm`, {}),
  runtimeDisarm: (droneId: string) => request("POST", `/api/runtime/drones/${droneId}/disarm`, {}),

  // ----- Guidance -----
  setDroneGuidance: (droneId: string, body: Record<string, unknown>) =>
    request("POST", `/api/guidance/drones/${droneId}`, body),
};

/** Service-relative live camera frame path for a drone in the active session. */
export function sessionFramePath(droneId: string): string {
  return `/api/session/drones/${encodeURIComponent(droneId)}/frame`;
}

/** Open the backend's gsplat orbit viewer for a stored splat record. */
export async function openSplatViewer(recordId: string): Promise<void> {
  openExternal(await absoluteServiceUrl(`/api/records/${recordId}/splat-viewer`));
}
