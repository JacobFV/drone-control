const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("droneStation", {
  request: (request) => ipcRenderer.invoke("app:request", request),
  serviceUrl: () => ipcRenderer.invoke("app:serviceUrl"),
});
