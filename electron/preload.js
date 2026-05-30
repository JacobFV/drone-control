const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("droneStation", {
  request: (request) => ipcRenderer.invoke("app:request", request),
  serviceUrl: () => ipcRenderer.invoke("app:serviceUrl"),
  wsUrl: () => ipcRenderer.invoke("app:wsUrl"),
  openExternal: (url) => ipcRenderer.invoke("app:openExternal", url),
  // Open a local folder/file in the OS file manager (e.g. a drone's record dir).
  openPath: (path) => ipcRenderer.invoke("app:openPath", path),
  // Binary fetch for endpoints the JSON IPC channel cannot carry (e.g. the
  // world-model .ply snapshot). Returns { ok, data: base64, mime } | { ok:false, error }.
  fetchBinary: (servicePath) => ipcRenderer.invoke("app:fetchBinary", servicePath),
});
