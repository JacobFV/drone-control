const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("droneStation", {
  getInitialState: () => ipcRenderer.invoke("app:getInitialState"),
});
