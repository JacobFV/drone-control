import { bridge } from "./bridge";
import type {
  ConfigStatus,
  DiscoverResult,
  Flight,
  ManualStatus,
  NetworkSummary,
  PoseTrackResult,
  ReconstructionStatus,
  ReconstructionTools,
  RuntimeStatus,
  SessionStatus,
  SimStatus,
  StationState,
  TrajectoriesResult,
  WorldSplatStatus,
} from "./types";

let serviceUrlCache = "";

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

/** Resolve a service-relative path against the (dynamic) service base URL. */
export async function absoluteServiceUrl(path: string): Promise<string> {
  const base = await getServiceUrl();
  return new URL(path, base).toString();
}

export function openExternal(url: string): void {
  if (bridge.openExternal) {
    void bridge.openExternal(url);
  } else {
    window.open(url, "_blank", "noopener");
  }
}

/**
 * Issue a request through the Electron IPC bridge. Returns null on failure so
 * callers can keep the original renderer's "best effort" semantics. Health is
 * tracked as a side effect.
 */
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

// Typed convenience wrappers for every backend endpoint the UI uses.
export const api = {
  // ----- State / config -----
  getState: () => request<StationState>("GET", "/api/state"),
  getConfig: () => request<ConfigStatus>("GET", "/api/config"),
  getNetwork: () => request<NetworkSummary>("GET", "/api/system/network"),
  getWifiCapabilities: () => request("GET", "/api/wifi/capabilities"),
  getWifiInterfaces: () => request("GET", "/api/wifi/interfaces"),
  getReconstructionTools: () =>
    request<ReconstructionTools>("GET", "/api/reconstruction/tools"),

  // ----- Discovery / wifi -----
  discoverDrones: (iface: string) =>
    request<DiscoverResult>("POST", "/api/drones/discover", { iface, rescan: true }),
  connectWifi: (iface: string, ssid: string, password: string) =>
    request<{ ok?: boolean }>("POST", "/api/wifi/connect", {
      iface,
      ssid,
      password,
      confirmDisconnect: true,
    }),
  reconnectWifi: (iface: string, password: string) =>
    request<{ ok?: boolean }>("POST", "/api/wifi/reconnect", { iface, password }),

  // ----- Manual control -----
  getManualStatus: () => request<ManualStatus>("GET", "/api/manual/status"),
  getManualConfig: () => request("GET", "/api/manual/config"),
  manualArm: () => request<ManualStatus>("POST", "/api/manual/arm", {}),
  manualDisarm: () => request<ManualStatus>("POST", "/api/manual/disarm", {}),
  manualClearFault: () => request<ManualStatus>("POST", "/api/manual/clear-fault", {}),
  manualStop: () => request<ManualStatus>("POST", "/api/manual/stop", {}),
  manualHeartbeat: () => request<ManualStatus>("POST", "/api/manual/heartbeat", {}),
  manualAxes: (axes: Record<string, number>) =>
    request<ManualStatus>("POST", "/api/manual/axes", axes),
  manualConfig: (config: Record<string, unknown>) =>
    request<ConfigStatus>("POST", "/api/manual/config", config),

  // ----- Runtime / swarm -----
  getRuntimeStatus: () => request<RuntimeStatus>("GET", "/api/runtime/status"),
  getRuntimeEvents: (since: number) =>
    request("GET", `/api/runtime/events?since=${since}`),
  runtimeStart: () => request<RuntimeStatus>("POST", "/api/runtime/start", {}),
  runtimeStop: () => request<RuntimeStatus>("POST", "/api/runtime/stop", {}),
  runtimeSetAllControllers: (mode: string) =>
    request<RuntimeStatus>("POST", "/api/runtime/controller", { mode }),
  runtimeSetController: (droneId: string, mode: string) =>
    request<RuntimeStatus>("POST", `/api/runtime/drones/${droneId}/controller`, { mode }),
  runtimeArm: (droneId: string) =>
    request<RuntimeStatus>("POST", `/api/runtime/drones/${droneId}/arm`, {}),
  runtimeDisarm: (droneId: string) =>
    request<RuntimeStatus>("POST", `/api/runtime/drones/${droneId}/disarm`, {}),
  runtimeHeartbeat: (droneId: string) =>
    request<RuntimeStatus>("POST", `/api/runtime/drones/${droneId}/heartbeat`, {}),
  runtimeAxes: (droneId: string, axes: Record<string, number>) =>
    request<RuntimeStatus>("POST", `/api/runtime/drones/${droneId}/axes`, axes),
  runtimeStopDrone: (droneId: string) =>
    request<RuntimeStatus>("POST", `/api/runtime/drones/${droneId}/stop`, {}),
  runtimeClearFault: (droneId: string) =>
    request<RuntimeStatus>("POST", `/api/runtime/drones/${droneId}/clear-fault`, {}),

  // ----- Mission -----
  missionStart: (body: Record<string, unknown> = {}) =>
    request("POST", "/api/mission/start", body),
  missionStop: () => request("POST", "/api/mission/stop", {}),

  // ----- Flights -----
  createFlight: (droneId: string, name: string) =>
    request<Flight>("POST", "/api/flights", { droneId, name, mode: "manual" }),
  updateFlight: (flightId: string, patch: Record<string, unknown>) =>
    request<Flight>("PATCH", `/api/flights/${flightId}`, patch),
  importRecord: (flightId: string, body: Record<string, unknown>) =>
    request<{ id: string }>("POST", `/api/flights/${flightId}/records`, body),

  // ----- Sessions / recording -----
  getSession: (flightId: string) =>
    request<SessionStatus>("GET", `/api/flights/${flightId}/session`),
  sessionStart: (flightId: string) =>
    request<SessionStatus>("POST", `/api/flights/${flightId}/session/start`, { source: "live" }),
  sessionStop: (flightId: string) =>
    request<SessionStatus>("POST", `/api/flights/${flightId}/session/stop`, {}),

  // ----- Pose track -----
  getPoseTrack: (flightId: string, since: number) =>
    request<PoseTrackResult>("GET", `/api/flights/${flightId}/pose/track?since=${since}`),
  computePoseTrack: (flightId: string) =>
    request("POST", `/api/flights/${flightId}/pose/compute`, {}),

  // ----- Reconstruction -----
  getReconstructionStatus: (flightId: string) =>
    request<ReconstructionStatus>("GET", `/api/flights/${flightId}/reconstruction/status`),
  reconstructionStart: (flightId: string, body: Record<string, unknown>) =>
    request<ReconstructionStatus>("POST", `/api/flights/${flightId}/reconstruction/start`, body),
  reconstructionStop: (flightId: string) =>
    request<ReconstructionStatus>("POST", `/api/flights/${flightId}/reconstruction/stop`, {}),

  // ----- Records -----
  recordExport: (recordId: string, format: string) =>
    request<{ id: string }>("POST", `/api/records/${recordId}/export`, { format, fps: 12 }),
  recordReveal: (recordId: string) =>
    request("POST", `/api/records/${recordId}/reveal`, {}),
  recordSplatViewer: async (recordId: string) => {
    openExternal(await absoluteServiceUrl(`/api/records/${recordId}/splat-viewer`));
  },

  // ----- World model / splat -----
  getWorldSplatStatus: () => request<WorldSplatStatus>("GET", "/api/world/splat/status"),
  worldSplatStart: () => request<WorldSplatStatus>("POST", "/api/world/splat/start", {}),
  worldSplatStop: () => request<WorldSplatStatus>("POST", "/api/world/splat/stop", {}),
  worldSplatBootstrap: (transforms: Record<string, unknown>) =>
    request<WorldSplatStatus>("POST", "/api/world/splat/bootstrap", { transforms }),
  worldSplatBootstrapFlights: (flightIds: string[]) =>
    request<WorldSplatStatus>("POST", "/api/world/splat/bootstrap", { flightIds }),

  // ----- Camera ingestion -----
  cameraStart: (droneId: string, body: Record<string, unknown>) =>
    request("POST", `/api/runtime/drones/${droneId}/camera/start`, body),
  cameraStop: (droneId: string) =>
    request("POST", `/api/runtime/drones/${droneId}/camera/stop`, {}),

  // ----- Simulation (live) -----
  getSimStatus: () => request<SimStatus>("GET", "/api/sim/status"),
  getSimTrajectories: () => request<TrajectoriesResult>("GET", "/api/sim/trajectories"),
  simStart: (body: Record<string, unknown>) => request<SimStatus>("POST", "/api/sim/start", body),
  simStop: () => request<SimStatus>("POST", "/api/sim/stop", {}),

  // ----- Multi-drone trajectories (real runtime) -----
  getRuntimeTrajectories: () => request<TrajectoriesResult>("GET", "/api/runtime/trajectories"),

  // ----- Guidance -----
  setDroneGuidance: (droneId: string, body: Record<string, unknown>) =>
    request("POST", `/api/guidance/drones/${droneId}`, body),
};

/** Service-relative per-drone camera frame paths (used as <img> src). */
export function simFramePath(index: number): string {
  return `/api/sim/drones/${index}/frame`;
}

export function runtimeFramePath(droneId: string): string {
  return `/api/runtime/drones/${droneId}/frame`;
}

/**
 * Download the live world-model .ply snapshot. The snapshot endpoint returns a
 * binary body, which the JSON IPC bridge cannot carry, so this uses the
 * fetchBinary bridge helper added to preload.js. Falls back to opening the URL
 * externally if the helper is unavailable.
 */
export async function downloadWorldSnapshot(): Promise<{ ok: boolean; error?: string }> {
  const path = "/api/world/splat/snapshot";
  if (bridge.fetchBinary) {
    const result = await bridge.fetchBinary(path);
    if (!result.ok || !result.data) return { ok: false, error: result.error ?? "snapshot failed" };
    const byteString = atob(result.data);
    const bytes = new Uint8Array(byteString.length);
    for (let i = 0; i < byteString.length; i += 1) bytes[i] = byteString.charCodeAt(i);
    const blob = new Blob([bytes], { type: result.mime ?? "application/octet-stream" });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = "world.ply";
    document.body.appendChild(anchor);
    anchor.click();
    anchor.remove();
    URL.revokeObjectURL(url);
    return { ok: true };
  }
  openExternal(await absoluteServiceUrl(path));
  return { ok: true };
}
