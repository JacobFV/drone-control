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
  openExternal: (url: string) => Promise<void>;
  // Optional binary fetch, added to preload.js to support the .ply snapshot.
  fetchBinary?: (path: string) => Promise<SnapshotResult>;
}

declare global {
  interface Window {
    droneStation: DroneStationBridge;
  }
}

export const bridge = window.droneStation;
