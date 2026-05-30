// Offscreen UI recorder. Renders the real control-station UI to an in-memory
// bitmap (Electron offscreen rendering) and writes a JPEG frame sequence — fully
// independent of any X display, so footage is clean even on a shared desktop.
//
// It points the UI at an already-running service (DRONE_SERVICE_URL/DRONE_WS_URL),
// drives a sim session over the service API, and scripts a shot list: the full
// tile wall, then each tile maximized in turn (via the `#max=<id>` hash). Frames
// land in <OUTDIR>/<phase>/fNNNNN.jpg for ffmpeg to encode afterwards.
const { app, BrowserWindow, ipcMain, shell } = require("electron");
const path = require("path");
const fs = require("fs");

const rootDir = path.resolve(__dirname, "..");
const serviceUrl = process.env.DRONE_SERVICE_URL;
const serviceWsUrl = process.env.DRONE_WS_URL || "";
const outDir = process.env.REC_OUT || path.join(rootDir, "film/assets/clips/ui/_frames");
const scene = process.env.REC_SCENE || "warehouse";
const drones = Number(process.env.REC_DRONES || 4);
const fps = Number(process.env.REC_FPS || 30);
const W = Number(process.env.REC_W || 1920);
const H = Number(process.env.REC_H || 1080);

app.disableHardwareAcceleration();
app.commandLine.appendSwitch("disable-gpu");

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function api(pathName, method = "GET", body) {
  const options = { method, headers: { "Content-Type": "application/json" } };
  if (body !== undefined) options.body = JSON.stringify(body);
  const res = await fetch(new URL(pathName, serviceUrl), options);
  return res.json().catch(() => ({}));
}

// ---- IPC the UI's preload expects -----------------------------------------
ipcMain.handle("app:serviceUrl", () => serviceUrl);
ipcMain.handle("app:wsUrl", () => serviceWsUrl);
ipcMain.handle("app:openExternal", async (_e, url) => shell.openExternal(url));
ipcMain.handle("app:request", async (_e, request) => api(request?.path || "/api/state", request?.method || "GET", request?.body));
ipcMain.handle("app:fetchBinary", async (_e, servicePath) => {
  try {
    const res = await fetch(new URL(servicePath, serviceUrl), { method: "GET" });
    if (!res.ok) return { ok: false, error: `${res.status} ${res.statusText}` };
    const buf = Buffer.from(await res.arrayBuffer());
    return { ok: true, data: buf.toString("base64"), mime: res.headers.get("content-type") || "application/octet-stream" };
  } catch (error) {
    return { ok: false, error: String(error) };
  }
});

let latest = null; // most recent NativeImage from the offscreen paint stream
let writer = null;  // { dir, idx } while a phase is being captured

app.whenReady().then(async () => {
  const win = new BrowserWindow({
    width: W,
    height: H,
    show: false,
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: false,
      offscreen: true,
    },
  });
  win.webContents.setFrameRate(fps);
  win.webContents.on("paint", (_e, _dirty, image) => {
    latest = image;
  });
  win.loadFile(path.join(rootDir, "ui", "dist", "index.html"));

  // Steady-cadence writer: independent of paint timing, so every phase gets a
  // predictable frame count even when the UI is momentarily static.
  setInterval(() => {
    if (!writer || !latest) return;
    const f = path.join(writer.dir, `f${String(writer.idx).padStart(5, "0")}.jpg`);
    try {
      fs.writeFileSync(f, latest.toJPEG(92));
      writer.idx += 1;
    } catch (_error) {
      /* ignore transient write races */
    }
  }, Math.round(1000 / fps));

  const setHash = (h) => win.webContents.executeJavaScript(`window.location.hash=${JSON.stringify(h)}`).catch(() => {});

  async function record(phase, seconds) {
    const dir = path.join(outDir, phase);
    fs.rmSync(dir, { recursive: true, force: true });
    fs.mkdirSync(dir, { recursive: true });
    writer = { dir, idx: 0 };
    await sleep(seconds * 1000);
    const count = writer.idx;
    writer = null;
    console.log(`PHASE ${phase} frames=${count}`);
  }

  try {
    await sleep(4000); // let the UI mount + connect to the service
    await api("/api/session/stop", "POST").catch(() => {});
    await sleep(800);
    await api("/api/session/start", "POST", {
      kind: "sim",
      name: "film",
      options: { numDrones: drones, task: "goto", scene, cameraModel: "ov2640", cameraNoise: "medium", maxSpeed: false, record: true },
    });
    await sleep(9000); // warm up: frames, trajectories, depth, point cloud

    await setHash("");
    await record("wall", 24);

    const tiles = ["omniscient", "camera-sim-0", "estimated-trajectory", "trajectory", "pointcloud", "seg-sim-0", "depth-sim-0", "world-seg"];
    for (const t of tiles) {
      await setHash(`#max=${t}`);
      await sleep(1500);
      await record(`tile_${t.replace(/[^a-z0-9]/gi, "_")}`, 14);
      await setHash("");
      await sleep(800);
    }

    await api("/api/session/stop", "POST").catch(() => {});
    console.log("REC_DONE");
  } catch (error) {
    console.error("REC_ERROR", error);
  } finally {
    app.quit();
  }
});

app.on("window-all-closed", () => app.quit());
