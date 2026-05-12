const { app, BrowserWindow, ipcMain } = require("electron");
const path = require("path");

const rootDir = path.resolve(__dirname, "..");

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

app.whenReady().then(() => {
  ipcMain.handle("app:getInitialState", () => getInitialState());
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

function getInitialState() {
  return {
    drones: [
      {
        id: "wifi8k-0c5b90",
        name: "WIFI_8K-0c5b90",
        model: "WIFI_8K",
        status: "available",
        lastSeen: "2026-05-12 15:09",
        connection: {
          ssid: "WIFI_8K-0c5b90",
          iface: "wlP9s9",
          ip: "192.168.1.1",
          control: "UDP 7099",
          camera: "RTSP 7070",
        },
        flights: [
          {
            id: "flight-20260512-150951",
            name: "Camera capture 15:09",
            startedAt: "2026-05-12 15:09:51",
            duration: "00:00:06",
            mode: "review",
            policy: "Manual camera capture",
            metadata: {
              battery: "fresh test battery",
              location: "bench",
              notes: "Autonomous RTSP camera startup verified.",
            },
            metrics: {
              packets: 1635,
              bytes: 2235749,
              frames: 126,
              resolution: "640 x 384",
              temporalMae: 3.012,
              smoothedTemporalMae: 2.145,
            },
            records: [
              { type: "frames", label: "Decoded forward JPEG frames", path: "camera_captures/frames_20260512_150951" },
              { type: "raw", label: "Raw UDP payloads", path: "camera_captures/camera_udp_20260512_150951.bin" },
              { type: "log", label: "Camera session log", path: "logs/drone_camera_session_20260512_150950.log" },
            ],
          },
          {
            id: "flight-20260512-145202",
            name: "Phone camera sniff 14:52",
            startedAt: "2026-05-12 14:52:02",
            duration: "00:01:00",
            mode: "review",
            policy: "Passive monitor capture",
            metadata: {
              source: "monitor pcap",
              notes: "RTSP negotiation was discovered from this capture.",
            },
            metrics: {
              packets: 4114,
              bytes: 5420000,
              frames: 326,
              resolution: "640 x 384",
              temporalMae: 10.922,
              smoothedTemporalMae: 7.463,
            },
            records: [
              { type: "pcap", label: "Monitor capture", path: "captures/drone_monitor_20260512_145202_ch1.pcap" },
              { type: "frames", label: "Decoded JPEG frames", path: "camera_captures/pcap_20260512_145202_jpeg_test" },
              { type: "frames", label: "Smoothed JPEG frames", path: "camera_captures/pcap_20260512_145202_smooth_fast_test" },
            ],
          },
        ],
      },
      {
        id: "wifi8k-second",
        name: "Second WIFI_8K drone",
        model: "WIFI_8K",
        status: "offline",
        lastSeen: "2026-05-12 12:51",
        connection: {
          ssid: "unknown",
          iface: "wlP9s9",
          ip: "192.168.1.1",
          control: "UDP 7099",
          camera: "RTSP 7070",
        },
        flights: [],
      },
    ],
  };
}
