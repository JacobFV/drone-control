// The window.droneStation bridge is defined by electron/preload.js. The React
// app must route ALL backend calls through it (never direct fetch) so that the
// Electron main process owns the dynamically-assigned service URL.

export interface DroneStationRequest {
  method: string;
  path: string;
  body?: unknown;
}

export interface SnapshotResult {
  ok: boolean;
  // base64-encoded body when ok; error text otherwise.
  data?: string;
  mime?: string;
  error?: string;
}

export interface DroneStationBridge {
  request: (request: DroneStationRequest) => Promise<unknown>;
  serviceUrl: () => Promise<string>;
  // WebSocket base URL announced by the Python service (empty if WS unavailable).
  wsUrl?: () => Promise<string>;
  openExternal: (url: string) => Promise<void>;
  // Open a local filesystem path (e.g. a drone's recording folder) in the OS
  // file manager. Resolves to "" on success or an error string.
  openPath?: (path: string) => Promise<string>;
  // Optional binary fetch, added to preload.js to support the .ply snapshot.
  fetchBinary?: (path: string) => Promise<SnapshotResult>;
}

declare global {
  interface Window {
    droneStation: DroneStationBridge;
  }
}

export const bridge = window.droneStation;
