const { app, BrowserWindow, Menu, ipcMain, shell } = require("electron");
const { spawn } = require("child_process");
const path = require("path");
const readline = require("readline");

const rootDir = path.resolve(__dirname, "..");
let serviceProcess = null;
let serviceUrl = "";
let serviceWsUrl = "";

app.disableHardwareAcceleration();
app.commandLine.appendSwitch("disable-gpu");

function createWindow() {
  // Recording mode (DRONE_REC): frameless window pinned to the screen origin at
  // a fixed size so an external screen-grab can capture a clean, stable frame.
  const rec = process.env.DRONE_REC;
  const recBounds = rec
    ? {
        x: 0,
        y: 0,
        width: Number(process.env.DRONE_REC_W || 1920),
        height: Number(process.env.DRONE_REC_H || 1080),
        frame: false,
      }
    : { width: 1440, height: 940, titleBarStyle: "hiddenInset" };
  const window = new BrowserWindow({
    ...recBounds,
    minWidth: 1120,
    minHeight: 720,
    title: "Drone Control Station",
    backgroundColor: "#111416",
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: false,
    },
  });

  // Single UI: the built React app in ui/dist. If it has not been built yet,
  // show a clear instruction page instead of silently loading something else.
  const builtUi = path.join(rootDir, "ui", "dist", "index.html");
  if (require("fs").existsSync(builtUi)) {
    window.loadFile(builtUi);
  } else {
    window.loadURL(
      "data:text/html;charset=utf-8," +
        encodeURIComponent(`<!doctype html><html><head><meta charset="utf-8">
<style>html,body{margin:0;height:100%;background:#0b0e10;color:#e7edf0;
font:14px/1.6 ui-monospace,Menlo,Consolas,monospace;display:flex;align-items:center;
justify-content:center}main{max-width:560px;padding:32px}h1{font-size:16px;
letter-spacing:.08em;text-transform:uppercase;color:#7fd1ff}code{background:#161b1f;
padding:2px 6px;border-radius:4px;color:#ffd35a}</style></head><body><main>
<h1>UI not built</h1><p>The React UI has not been built yet. Run:</p>
<p><code>npm --prefix ui install &amp;&amp; npm --prefix ui run build</code></p>
<p>then restart the app with <code>npm start</code>.</p></main></body></html>`),
    );
  }

  // Recording automation: poll a command file and run each new line as JS in the
  // renderer (e.g. set `location.hash` to maximize a tile). Keeps the recorder
  // in control of the live UI without simulated mouse/keyboard input.
  if (process.env.DRONE_REC) {
    const cmdFile = process.env.DRONE_REC_CMD || "/tmp/drone_rec_cmd";
    let last = "";
    setInterval(() => {
      let text = "";
      try {
        text = require("fs").readFileSync(cmdFile, "utf8");
      } catch (_error) {
        return;
      }
      if (text === last) return;
      last = text;
      const js = text.trim();
      if (js) window.webContents.executeJavaScript(js).catch(() => {});
    }, 200);
  }
}

app.whenReady().then(async () => {
  Menu.setApplicationMenu(null);
  // Allow pointing the UI at an already-running service (recording/automation),
  // instead of spawning a fresh one on a random port.
  if (process.env.DRONE_SERVICE_URL) {
    serviceUrl = process.env.DRONE_SERVICE_URL;
    serviceWsUrl = process.env.DRONE_WS_URL || "";
  } else {
    serviceUrl = await startPythonService();
  }
  ipcMain.handle("app:request", async (_event, request) => serviceRequest(request));
  ipcMain.handle("app:serviceUrl", () => serviceUrl);
  ipcMain.handle("app:wsUrl", () => serviceWsUrl);
  ipcMain.handle("app:openExternal", async (_event, url) => shell.openExternal(url));
  ipcMain.handle("app:fetchBinary", async (_event, servicePath) => serviceFetchBinary(servicePath));
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
      if (line.startsWith("WS_READY ")) {
        serviceWsUrl = line.replace("WS_READY ", "").trim();
        return;
      }
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
    let detail = "";
    try {
      const payload = await response.json();
      if (payload && typeof payload.error === "string") detail = `: ${payload.error}`;
    } catch (_error) {
      detail = "";
    }
    throw new Error(`Service request failed: ${response.status} ${response.statusText}${detail}`);
  }
  return response.json();
}

async function serviceFetchBinary(servicePath) {
  try {
    const response = await fetch(new URL(servicePath, serviceUrl), { method: "GET" });
    if (!response.ok) {
      return { ok: false, error: `${response.status} ${response.statusText}` };
    }
    const buffer = Buffer.from(await response.arrayBuffer());
    return {
      ok: true,
      data: buffer.toString("base64"),
      mime: response.headers.get("content-type") || "application/octet-stream",
    };
  } catch (error) {
    return { ok: false, error: String(error) };
  }
}
