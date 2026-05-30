import { bridge } from "./bridge";
import type {
  AccessPoint,
  ConfigStatus,
  DiscoverResult,
  DroneDetail,
  ManualStatus,
  NetworkSummary,
  SerialBridge,
  SessionStatus,
  StationState,
  WifiInterface,
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

/** Open a local folder in the OS file manager (Electron only). */
export function openPath(path: string): void {
  if (bridge.openPath) void bridge.openPath(path);
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

export async function requestOrThrow<T = unknown>(
  method: string,
  path: string,
  body?: unknown,
): Promise<T> {
  try {
    const result = (await bridge.request({ method, path, body })) as T;
    setHealth("ready");
    return result;
  } catch (error) {
    console.error(`[droneStation] ${method} ${path}`, error);
    setHealth("error");
    throw error;
  }
}

export const api = {
  // ----- State / config -----
  getState: () => request<StationState>("GET", "/api/state"),
  getConfig: () => request<ConfigStatus>("GET", "/api/config"),
  getNetwork: () => request<NetworkSummary>("GET", "/api/system/network"),

  // ----- Scenes / environments -----
  getScenes: () => request<{ scenes: { id: string; name: string; kind: string }[] }>("GET", "/api/scenes"),
  getCameras: () =>
    request<{ cameras: { id: string; name: string; width: number; height: number; fps: number; hfovDeg: number; sensor: string }[] }>(
      "GET",
      "/api/cameras",
    ),
  createEnvironment: (name: string, kind: string, config: Record<string, unknown> = {}) =>
    request<{ id: string }>("POST", "/api/environments", { name, kind, config }),

  // ----- Records (review) -----
  getRecordPoseTrack: (recordId: string) =>
    request<{ poses: { x: number; y: number; z: number }[] }>("GET", `/api/records/${recordId}/pose-track`),
  getRecordArtifact: <T = unknown>(recordId: string) =>
    request<T>("GET", `/api/records/${recordId}/artifact`),

  // ----- Session (the single active flight session) -----
  getSessionStatus: () => request<SessionStatus>("GET", "/api/session/status"),
  sessionStart: (kind: string, name: string, options: Record<string, unknown>) =>
    request<SessionStatus>("POST", "/api/session/start", { kind, name, options }),
  sessionStop: () => request<SessionStatus>("POST", "/api/session/stop", {}),
  sessionSpeed: (mode: "realtime" | "max") =>
    request<SessionStatus>("POST", "/api/session/speed", { mode }),
  sessionPause: () => request<SessionStatus>("POST", "/api/session/pause", {}),
  sessionResume: () => request<SessionStatus>("POST", "/api/session/resume", {}),

  // Per-drone control (sim sessions): hold one drone in place / release it, and
  // fetch its command history + record directory.
  droneEstop: (droneId: string) =>
    request<SessionStatus>("POST", `/api/session/drones/${encodeURIComponent(droneId)}/estop`, {}),
  droneRelease: (droneId: string) =>
    request<SessionStatus>("POST", `/api/session/drones/${encodeURIComponent(droneId)}/release`, {}),
  getDroneDetail: (droneId: string) =>
    request<DroneDetail>("GET", `/api/session/drones/${encodeURIComponent(droneId)}/detail`),

  getPointCloud: (max = 2500) =>
    request<{ points: number[][] }>("GET", `/api/session/pointcloud?max=${max}`),

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
  getWifiInterfaces: () => request<{ interfaces: WifiInterface[] }>("GET", "/api/wifi/interfaces"),
  getAccessPoints: (iface: string, rescan = false) =>
    request<{ accessPoints: AccessPoint[] }>(
      "GET",
      `/api/wifi/access-points?iface=${encodeURIComponent(iface)}&rescan=${rescan ? "1" : "0"}`,
    ),
  getSerialBridges: () => request<{ bridges: SerialBridge[]; platform?: string }>("GET", "/api/serial/bridges"),

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

  // ----- LLM coordinator (high-level director) -----
  getCoordinatorConfig: () => request<CoordinatorConfigResult>("GET", "/api/coordinator/config"),
  setCoordinatorConfig: (body: Record<string, unknown>) =>
    request<CoordinatorConfigResult>("POST", "/api/coordinator/config", body),
  missionStart: (objective: string) =>
    request("POST", "/api/mission/start", { objective, controllerMode: "batched_vla" }),
  missionStop: () => request("POST", "/api/mission/stop", {}),

  // ----- VLA model registry -----
  getModels: () => request<ModelsResult>("GET", "/api/models"),
  downloadModel: (id: string) => request<ModelsResult>("POST", `/api/models/${id}/download`, {}),
  downloadModelOrThrow: (id: string) => requestOrThrow<ModelsResult>("POST", `/api/models/${id}/download`, {}),
  selectModel: (id: string | null) => request<ModelsResult>("POST", "/api/models/select", { id }),
};

export interface ModelEntry {
  id: string;
  name: string;
  kind: string;
  description: string;
  hfRepo: string;
  ghUrl: string;
  params: string;
  downloaded: boolean;
  sizeBytes: number;
  active: boolean;
}
export interface ModelsResult {
  models: ModelEntry[];
  active: string | null;
}

export interface CoordinatorConfigResult {
  config: {
    provider: string;
    model: string;
    baseUrl: string;
    temperature: number;
    maxTokens: number;
    hasApiKey: boolean;
    configured: boolean;
  };
  lastError?: string | null;
  mission?: { state?: string; notes?: string[]; toolCalls?: { name: string; arguments: Record<string, unknown> }[] };
  guidance?: Record<string, unknown>;
}

/** Service-relative live camera frame path for a drone in the active session. */
export function sessionFramePath(droneId: string): string {
  return `/api/session/drones/${encodeURIComponent(droneId)}/frame`;
}

/** Service-relative colorized depth-map frame path for a drone. */
export function sessionDepthPath(droneId: string): string {
  return `/api/session/drones/${encodeURIComponent(droneId)}/depth`;
}

/** Service-relative omniscient god's-eye view of the sim world (sim sessions only). */
export function sessionOmniscientPath(): string {
  return `/api/session/omniscient`;
}

/** Open the backend's gsplat orbit viewer for a stored splat record. */
export async function openSplatViewer(recordId: string): Promise<void> {
  openExternal(await absoluteServiceUrl(`/api/records/${recordId}/splat-viewer`));
}
