const { app, BrowserWindow, Menu, ipcMain, shell } = require("electron");
const { spawn } = require("child_process");
const path = require("path");
const readline = require("readline");

const rootDir = path.resolve(__dirname, "..");
let serviceProcess = null;
let serviceUrl = "";

app.disableHardwareAcceleration();
app.commandLine.appendSwitch("disable-gpu");

function createWindow() {
  const window = new BrowserWindow({
    width: 1440,
    height: 940,
    minWidth: 1120,
    minHeight: 720,
    title: "Drone Control Station",
    backgroundColor: "#111416",
    titleBarStyle: "hiddenInset",
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: false,
    },
  });

  window.loadFile(path.join(rootDir, "app", "index.html"));
}

app.whenReady().then(async () => {
  Menu.setApplicationMenu(null);
  serviceUrl = await startPythonService();
  ipcMain.handle("app:request", async (_event, request) => serviceRequest(request));
  ipcMain.handle("app:serviceUrl", () => serviceUrl);
  ipcMain.handle("app:openExternal", async (_event, url) => shell.openExternal(url));
  createWindow();

  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow();
    }
  });
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") {
    app.quit();
  }
});

app.on("before-quit", () => {
  if (serviceProcess && !serviceProcess.killed) {
    serviceProcess.kill();
  }
});

async function startPythonService() {
  return new Promise((resolve, reject) => {
    const venvPython = process.platform === "win32"
      ? path.join(rootDir, ".venv", "Scripts", "python.exe")
      : path.join(rootDir, ".venv", "bin", "python");
    const python = process.env.PYTHON || (require("fs").existsSync(venvPython) ? venvPython : "python3");
    const pythonBinDir = path.dirname(python);
    const env = { ...process.env };
    env.PATH = `${pythonBinDir}${path.delimiter}${env.PATH || ""}`;
    serviceProcess = spawn(python, ["-m", "drone_control.service", "--host", "127.0.0.1", "--port", "0"], {
      cwd: rootDir,
      env,
      stdio: ["ignore", "pipe", "pipe"],
    });

    const timeout = setTimeout(() => {
      reject(new Error("Timed out waiting for Python service startup"));
    }, 15000);

    const stdout = readline.createInterface({ input: serviceProcess.stdout });
    stdout.on("line", (line) => {
      if (line.startsWith("SERVICE_READY ")) {
        clearTimeout(timeout);
        resolve(line.replace("SERVICE_READY ", "").trim());
      }
    });

    serviceProcess.stderr.on("data", (chunk) => {
      process.stderr.write(chunk);
    });

    serviceProcess.on("exit", (code, signal) => {
      if (!serviceUrl) {
        clearTimeout(timeout);
        reject(new Error(`Python service exited before startup: code=${code} signal=${signal}`));
      }
    });
  });
}

async function serviceRequest(request) {
  const method = request?.method || "GET";
  const pathName = request?.path || "/api/state";
  const options = { method, headers: { "Content-Type": "application/json" } };
  if (request?.body !== undefined) {
    options.body = JSON.stringify(request.body);
  }
  const response = await fetch(new URL(pathName, serviceUrl), options);
  if (!response.ok) {
    throw new Error(`Service request failed: ${response.status} ${response.statusText}`);
  }
  return response.json();
}
