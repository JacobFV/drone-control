const KNOWN_STATUS = new Set(["available", "offline"]);

const state = {
  drones: [],
  selectedDroneId: "",
  selectedFlightId: "",
  mainView: "forward",
  lhsCollapsed: false,
  rhsCollapsed: false,
  mode: "review",
  serviceUrl: "",
  service: "STARTING",
  manualStatus: null,
  sessionStatus: null,
  heartbeatTimer: null,
  refreshTimer: null,
  treeSignature: "",
  config: null,
  network: null,
  selectedRecordId: "",
};

const workspace = document.querySelector(".workspace");
const droneTree = document.getElementById("droneTree");
const droneCount = document.getElementById("droneCount");
const metadataList = document.getElementById("metadataList");
const metricsGrid = document.getElementById("metricsGrid");
const recordsList = document.getElementById("recordsList");
const manualPanel = document.getElementById("manualPanel");
const throttle = document.getElementById("throttle");
const throttleValue = document.getElementById("throttleValue");
const forwardStream = document.getElementById("forwardStream");
const forwardEmpty = document.getElementById("forwardEmpty");
const forwardResolution = document.getElementById("forwardResolution");
const forwardEndpoint = document.getElementById("forwardEndpoint");
const serviceStatus = document.getElementById("serviceStatus");
const manualState = document.getElementById("manualState");
const manualMessage = document.getElementById("manualMessage");
const armButton = document.getElementById("armButton");
const disarmButton = document.getElementById("disarmButton");
const stopButton = document.getElementById("stopButton");
const recordToggle = document.getElementById("recordToggle");
const recordingCounter = document.getElementById("recordingCounter");
const lhsToggle = document.getElementById("lhsToggle");
const rhsToggle = document.getElementById("rhsToggle");
const platformState = document.getElementById("platformState");
const networkList = document.getElementById("networkList");
const ifaceSelect = document.getElementById("ifaceSelect");
const ssidInput = document.getElementById("ssidInput");
const wifiPassword = document.getElementById("wifiPassword");
const scanButton = document.getElementById("scanButton");
const connectButton = document.getElementById("connectButton");
const reconnectButton = document.getElementById("reconnectButton");
const networkMessage = document.getElementById("networkMessage");
const policyPanel = document.getElementById("policyPanel");
const policyMaxThrottle = document.getElementById("policyMaxThrottle");
const policyCommandHz = document.getElementById("policyCommandHz");
const policySlew = document.getElementById("policySlew");
const policyHeartbeat = document.getElementById("policyHeartbeat");
const savePolicyButton = document.getElementById("savePolicyButton");
const ioState = document.getElementById("ioState");
const ioEnabled = document.getElementById("ioEnabled");
const manualIface = document.getElementById("manualIface");
const manualIp = document.getElementById("manualIp");
const manualPort = document.getElementById("manualPort");
const manualProtocol = document.getElementById("manualProtocol");
const manualBindDevice = document.getElementById("manualBindDevice");
const saveManualConfig = document.getElementById("saveManualConfig");
const importPath = document.getElementById("importPath");
const importFramesButton = document.getElementById("importFramesButton");
const exportMjpegButton = document.getElementById("exportMjpegButton");
const exportMp4Button = document.getElementById("exportMp4Button");

init();

async function init() {
  state.serviceUrl = await window.droneStation.serviceUrl();
  const initial = await loadState();
  state.config = await safeRequest("GET", "/api/config");
  state.network = await safeRequest("GET", "/api/system/network");
  if (initial) {
    state.drones = initial.drones;
    state.selectedDroneId = state.drones[0]?.id ?? "";
    state.selectedFlightId = state.drones[0]?.flights[0]?.id ?? "";
  }
  await refreshManualStatus();
  await refreshSessionStatus();
  render();
  wireToolbar();
  wireModeSelector();
  wireControls();
  wireNetwork();
  wirePolicy();
  wireManualConfig();
  wireRecordActions();
  state.refreshTimer = window.setInterval(refreshAppState, 5000);
}

function wireToolbar() {
  lhsToggle.addEventListener("click", () => {
    state.lhsCollapsed = !state.lhsCollapsed;
    lhsToggle.classList.toggle("is-active", !state.lhsCollapsed);
    applyLayout();
  });

  rhsToggle.addEventListener("click", () => {
    state.rhsCollapsed = !state.rhsCollapsed;
    rhsToggle.classList.toggle("is-active", !state.rhsCollapsed);
    applyLayout();
  });

  document.querySelectorAll(".segmented [data-view]").forEach((button) => {
    button.addEventListener("click", () => {
      state.mainView = button.dataset.view;
      renderMainView();
    });
  });

  document.getElementById("newFlight").addEventListener("click", createDraftFlight);
  recordToggle.addEventListener("click", toggleRecording);
}

function wireModeSelector() {
  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      const next = button.dataset.mode;
      const prev = state.mode;
      if (next === prev) return;
      state.mode = next;
      renderMode();
      persistSelectedFlightMode(next);
      if (prev === "manual" && next !== "manual") {
        safeRequest("POST", "/api/manual/disarm", {}).then((s) => s && updateManualStatus(s));
      }
    });
  });
}

function wireControls() {
  throttle.addEventListener("input", () => {
    throttleValue.textContent = throttle.value;
    sendManualAxes({ throttle: Number(throttle.value) });
  });

  document.querySelectorAll("[data-control]").forEach((button) => {
    button.addEventListener("click", () => {
      const control = button.dataset.control;
      if (control === "stop") {
        emergencyStop();
      } else {
        sendManualAxes(controlAxes(control));
      }
    });
  });

  armButton.addEventListener("click", async () => {
    if (state.manualStatus?.state === "faulted") {
      await safeRequest("POST", "/api/manual/clear-fault", {});
    }
    const status = await safeRequest("POST", "/api/manual/arm", {});
    if (status) updateManualStatus(status);
  });
  disarmButton.addEventListener("click", async () => {
    const status = await safeRequest("POST", "/api/manual/disarm", {});
    if (status) updateManualStatus(status);
  });
  stopButton.addEventListener("click", emergencyStop);
}

function wireNetwork() {
  scanButton.addEventListener("click", discoverDrones);
  connectButton.addEventListener("click", connectSelectedWifi);
  reconnectButton.addEventListener("click", reconnectWifi);
  ifaceSelect.addEventListener("change", () => {
    manualIface.value = ifaceSelect.value;
  });
}

function wirePolicy() {
  savePolicyButton.addEventListener("click", savePolicy);
}

function wireManualConfig() {
  saveManualConfig.addEventListener("click", saveManualIoConfig);
}

function wireRecordActions() {
  importFramesButton.addEventListener("click", importFrames);
  exportMjpegButton.addEventListener("click", () => exportSelectedRecord("mjpeg"));
  exportMp4Button.addEventListener("click", () => exportSelectedRecord("mp4"));
}

window.addEventListener("beforeunload", () => {
  if (state.heartbeatTimer !== null) window.clearInterval(state.heartbeatTimer);
  if (state.refreshTimer !== null) window.clearInterval(state.refreshTimer);
});

async function emergencyStop() {
  const status = await safeRequest("POST", "/api/manual/stop", {});
  if (!status) return;
  throttle.value = "0";
  throttleValue.textContent = "0";
  updateManualStatus(status);
}

async function sendManualAxes(axes) {
  if (state.mode !== "manual") return;
  const status = await safeRequest("POST", "/api/manual/axes", axes);
  if (status) updateManualStatus(status);
}

function controlAxes(control) {
  const center = 128;
  const step = 28;
  switch (control) {
    case "pitch-up":    return { pitch: center + step };
    case "pitch-down":  return { pitch: center - step };
    case "roll-left":   return { roll: center - step };
    case "roll-right":  return { roll: center + step };
    case "yaw-left":    return { yaw: center - step };
    case "yaw-right":   return { yaw: center + step };
    default:            return {};
  }
}

async function refreshManualStatus() {
  const status = await safeRequest("GET", "/api/manual/status");
  if (status) updateManualStatus(status);
}

function updateManualStatus(status) {
  state.manualStatus = status;
  renderManualStatus();
}

function renderManualStatus() {
  const status = state.manualStatus;
  if (!status) return;
  manualState.textContent = String(status.state).toUpperCase();
  manualState.classList.toggle("is-danger", status.state === "faulted");
  manualState.classList.toggle("is-armed", Boolean(status.armed));
  manualMessage.classList.toggle("is-danger", status.state === "faulted");
  manualMessage.textContent = manualMessageText(status);
  armButton.textContent = status.state === "faulted" ? "CLEAR + ARM" : "ARM";
  armButton.disabled = Boolean(status.armed);
  disarmButton.disabled = !status.armed;
  throttle.disabled = !status.armed;
  document.querySelectorAll("[data-control]").forEach((button) => {
    if (button.dataset.control !== "stop") button.disabled = !status.armed;
  });
}

function manualMessageText(status) {
  if (status.faultReason) return `FAULT: ${String(status.faultReason).toUpperCase()}`;
  if (status.stopReason) return `STOPPING: ${String(status.stopReason).toUpperCase()}`;
  const transport = status.transport;
  const armed = status.armed ? "ARMED · HEARTBEAT" : "DISARMED";
  if (!transport?.enabled) return `${armed} · IO DISABLED`;
  if (transport.lastError) return `${armed} · ${String(transport.lastError).toUpperCase()}`;
  return `${armed} · ${transport.connected ? "CONNECTED" : "READY"} · ${transport.target}`;
}

function render() {
  applyLayout();
  renderNetwork();
  renderManualConfig();
  renderTree();
  renderMainView();
  renderInspector();
  renderStream();
  renderServiceStatus();
}

function applyLayout() {
  workspace.classList.toggle("lhs-collapsed", state.lhsCollapsed);
  workspace.classList.toggle("rhs-collapsed", state.rhsCollapsed);
}

function treeSignature() {
  return state.drones
    .map((d) => `${d.id}:${d.status}:${d.name}:${d.model}:${d.lastSeen}:${d.flights.map((f) => `${f.id}/${f.mode}/${f.name}/${f.duration}`).join(",")}`)
    .join("|");
}

function renderTree() {
  droneCount.textContent = state.drones.length;
  const sig = treeSignature();
  if (sig === state.treeSignature) {
    document.querySelectorAll(".flight-row").forEach((row) => {
      row.classList.toggle("is-active", row.dataset.flightId === state.selectedFlightId);
    });
    return;
  }
  state.treeSignature = sig;
  droneTree.replaceChildren();

  state.drones.forEach((drone) => {
    const group = element("div", "tree-group is-expanded");
    const droneButton = element("button", "drone-row");
    const statusClass = KNOWN_STATUS.has(drone.status) ? drone.status : "unknown";
    droneButton.innerHTML = `
      <svg class="chevron" viewBox="0 0 12 12" aria-hidden="true"><path d="M3 4.5 L6 8 L9 4.5"/></svg>
      <span class="tree-name">${escapeHtml(drone.name)}<span class="tree-sub">${escapeHtml(drone.model)} · ${escapeHtml(drone.lastSeen)}</span></span>
      <span class="status-dot ${statusClass}"></span>
    `;
    droneButton.addEventListener("click", () => {
      group.classList.toggle("is-expanded");
      state.selectedDroneId = drone.id;
      if (!state.selectedFlightId && drone.flights[0]) {
        state.selectedFlightId = drone.flights[0].id;
      }
      refreshSessionStatus();
      renderInspector();
    });

    const children = element("div", "children");
    if (drone.flights.length === 0) {
      const empty = element("div", "flight-row");
      empty.innerHTML = `<span class="tree-name">—<span class="tree-sub">no flights</span></span><span></span>`;
      children.append(empty);
    }

    drone.flights.forEach((flight) => {
      const isActive = flight.id === state.selectedFlightId;
      const flightButton = element("button", `flight-row${isActive ? " is-active" : ""}`);
      flightButton.dataset.flightId = flight.id;
      flightButton.innerHTML = `
        <span class="tree-name">${escapeHtml(flight.name)}<span class="tree-sub">${escapeHtml(flight.duration)} · ${escapeHtml(flight.mode)}</span></span>
        <span></span>
      `;
      flightButton.addEventListener("click", () => {
        state.selectedDroneId = drone.id;
        state.selectedFlightId = flight.id;
        state.mode = flight.mode ?? "review";
        refreshSessionStatus();
        renderTree();
        renderInspector();
        renderStream();
      });
      children.append(flightButton);
    });

    group.append(droneButton, children);
    droneTree.append(group);
  });
}

function renderMainView() {
  document.querySelectorAll("[data-view]").forEach((button) => {
    button.classList.toggle("is-active", button.dataset.view === state.mainView);
  });
  document.querySelectorAll(".view-panel").forEach((panel) => panel.classList.remove("is-active"));
  document.getElementById(`${state.mainView}View`).classList.add("is-active");
  renderStream();
}

function renderInspector() {
  const drone = selectedDrone();
  const flight = selectedFlight();
  if (!drone) {
    metadataList.replaceChildren();
    metricsGrid.replaceChildren();
    recordsList.replaceChildren();
    renderRecordCounter();
    return;
  }

  if (flight) state.mode = flight.mode ?? state.mode;
  hydratePolicyForm(flight?.policy ?? {});
  renderMode();
  renderRecordToggle();
  renderRecordCounter();

  renderKv(metadataList, [
    ["Drone", drone.name],
    ["SSID", drone.connection.ssid],
    ["Iface", drone.connection.iface],
    ["IP", drone.connection.ip],
    ["Control", drone.connection.control, "mono"],
    ["Camera", drone.connection.camera, "mono"],
    ["Policy", formatPolicy(flight?.policy)],
    ["Started", flight?.startedAt ?? "—"],
    ...Object.entries(flight?.metadata ?? {}).map(([k, v]) => [k, v]),
  ]);

  const m = flight?.metrics ?? {};
  renderKv(metricsGrid, [
    ["Frames", m.frames],
    ["Packets", m.packets],
    ["Bytes", m.bytes],
    ["Resolution", m.resolution],
    ["MAE T", m.temporalMae],
    ["MAE T̄", m.smoothedTemporalMae],
  ].filter(([, v]) => v !== undefined && v !== null));

  renderRecords(flight?.records ?? []);
  renderStream();
}

function renderNetwork() {
  const network = state.network || state.config?.network;
  const interfaces = network?.interfaces ?? [];
  platformState.textContent = String(network?.platform ?? state.config?.platform ?? "—").toUpperCase();
  renderKv(networkList, [
    ["Default", network?.defaultInterface],
    ["Wi-Fi", interfaces.length],
    ["Mode", network?.singleWifiLikely ? "single radio" : "multi radio"],
  ]);
  const current = ifaceSelect.value;
  ifaceSelect.replaceChildren();
  interfaces.forEach((item) => {
    const option = element("option");
    option.value = item.name;
    option.textContent = `${item.name}${item.connection ? ` · ${item.connection}` : ""}`;
    ifaceSelect.append(option);
  });
  const fallback = network?.defaultInterface || state.config?.manual?.iface || "en0";
  if (!interfaces.some((item) => item.name === fallback)) {
    const option = element("option");
    option.value = fallback;
    option.textContent = fallback;
    ifaceSelect.append(option);
  }
  ifaceSelect.value = interfaces.some((item) => item.name === current) ? current : fallback;
  if (!ssidInput.value) ssidInput.value = selectedDrone()?.connection?.ssid ?? "";
  networkMessage.textContent = String(network?.notes ?? "ONE ACTIVE DRONE CONNECTION PER WI-FI RADIO").toUpperCase();
}

function renderManualConfig() {
  const manual = state.config?.manual || state.manualStatus?.transport || {};
  ioEnabled.checked = Boolean(manual.enabled);
  manualIface.value = manual.iface ?? ifaceSelect.value ?? "";
  manualIp.value = manual.ip ?? "192.168.1.1";
  manualPort.value = manual.port ?? 7099;
  manualProtocol.value = manual.protocol ?? "wifi_8k_prefixed_short";
  manualBindDevice.checked = Boolean(manual.bindDevice);
  ioState.textContent = manual.enabled ? "ON" : "OFF";
  ioState.classList.toggle("is-armed", Boolean(manual.enabled));
}

function hydratePolicyForm(policy) {
  if (!policy || typeof policy !== "object") return;
  policyMaxThrottle.value = policy.maxThrottle ?? policy.max_throttle ?? policyMaxThrottle.value;
  policyCommandHz.value = policy.commandHz ?? policy.command_hz ?? policyCommandHz.value;
  policySlew.value = policy.throttleSlewPerSecond ?? policy.throttle_slew_per_second ?? policySlew.value;
  policyHeartbeat.checked = policy.requireHeartbeat !== false;
}

function renderMode() {
  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.classList.toggle("is-active", button.dataset.mode === state.mode);
  });
  manualPanel.classList.toggle("is-hidden", state.mode !== "manual");
  policyPanel.classList.toggle("is-hidden", state.mode !== "policy");
  if (state.mode === "manual") startHeartbeat();
  else stopHeartbeat();
  renderManualStatus();
}

function renderKv(parent, entries) {
  parent.replaceChildren();
  entries.forEach(([key, value, mod]) => {
    const dt = element("dt");
    dt.textContent = key;
    const dd = element("dd", mod === "mono" ? "mono" : "");
    dd.textContent = value === undefined || value === null || value === "" ? "—" : formatValue(value);
    parent.append(dt, dd);
  });
}

function renderRecords(records) {
  recordsList.replaceChildren();
  if (records.length === 0) {
    recordsList.classList.add("empty");
    return;
  }
  recordsList.classList.remove("empty");
  records.forEach((record) => {
    const dt = element("dt");
    dt.textContent = record.label;
    const dd = element("dd", "mono");
    const path = element("div");
    path.textContent = record.path ?? record.blobKey ?? "—";
    const actions = element("div", "record-actions");
    const select = element("button", "btn");
    select.textContent = state.selectedRecordId === record.id ? "SELECTED" : "SELECT";
    select.addEventListener("click", () => {
      state.selectedRecordId = record.id;
      renderRecords(records);
    });
    const reveal = element("button", "btn");
    reveal.textContent = "SHOW";
    reveal.addEventListener("click", () => revealRecord(record.id));
    actions.append(select, reveal);
    dd.append(path, actions);
    recordsList.append(dt, dd);
  });
}

async function createDraftFlight() {
  const drone = selectedDrone();
  if (!drone) return;
  const now = new Date();
  const created = await safeRequest("POST", "/api/flights", {
    droneId: drone.id,
    name: `Draft ${now.toLocaleTimeString()}`,
  });
  if (!created) return;
  const refreshed = await loadState();
  if (refreshed) state.drones = refreshed.drones;
  state.selectedFlightId = created.id;
  state.mode = "manual";
  await refreshSessionStatus();
  renderTree();
  renderInspector();
}

function renderStream() {
  const drone = selectedDrone();
  const flight = selectedFlight();
  forwardEndpoint.textContent = drone?.connection?.camera ?? "—";

  const framesRecord = selectedFrameRecord();
  if (framesRecord && state.mainView === "forward") {
    const nextUrl = absoluteServiceUrl(`${framesRecord.streamUrl}?fps=12`);
    if (forwardStream.src !== nextUrl) forwardStream.src = nextUrl;
    forwardStream.classList.remove("is-hidden");
    forwardEmpty.classList.add("is-hidden");
    forwardResolution.textContent = flight?.metrics?.resolution ?? "—";
    return;
  }
  if (forwardStream.hasAttribute("src")) forwardStream.removeAttribute("src");
  forwardStream.classList.add("is-hidden");
  forwardEmpty.classList.remove("is-hidden");
  forwardResolution.textContent = flight?.metrics?.resolution ?? "—";
}

async function persistSelectedFlightMode(mode) {
  const flight = selectedFlight();
  if (!flight) return;
  const updated = await safeRequest("PATCH", `/api/flights/${flight.id}`, { mode });
  if (updated) flight.mode = updated.mode;
}

async function refreshAppState() {
  const refreshed = await safeRequest("GET", "/api/state");
  if (!refreshed) return;
  const prevDroneId = state.selectedDroneId;
  const prevFlightId = state.selectedFlightId;
  state.drones = refreshed.drones;
  state.selectedDroneId = state.drones.find((d) => d.id === prevDroneId)?.id ?? state.drones[0]?.id ?? "";
  state.selectedFlightId =
    selectedFlight()?.id ??
    state.drones.find((d) => d.id === state.selectedDroneId)?.flights[0]?.id ??
    prevFlightId;
  renderTree();
  renderInspector();
  await refreshSessionStatus();
}

async function refreshSessionStatus() {
  const flight = selectedFlight();
  if (!flight) {
    state.sessionStatus = null;
    renderRecordToggle();
    renderRecordCounter();
    return;
  }
  const status = await safeRequest("GET", `/api/flights/${flight.id}/session`);
  if (status) state.sessionStatus = status;
  renderRecordToggle();
  renderRecordCounter();
}

function renderRecordToggle() {
  const running = Boolean(state.sessionStatus?.running);
  const hasFlight = Boolean(selectedFlight());
  recordToggle.disabled = !hasFlight;
  recordToggle.textContent = running ? "STOP" : "RECORD";
  recordToggle.classList.toggle("is-danger", running);
}

function renderRecordCounter() {
  const running = Boolean(state.sessionStatus?.running);
  if (!running) {
    recordingCounter.classList.add("is-hidden");
    return;
  }
  recordingCounter.classList.remove("is-hidden");
  recordingCounter.textContent = `REC ${formatValue(state.sessionStatus.frames ?? 0)}`;
}

async function toggleRecording() {
  const flight = selectedFlight();
  if (!flight) return;
  const running = Boolean(state.sessionStatus?.running);
  const path = running
    ? `/api/flights/${flight.id}/session/stop`
    : `/api/flights/${flight.id}/session/start`;
  const body = running ? {} : { source: "live" };
  const status = await safeRequest("POST", path, body);
  if (!status) return;
  state.sessionStatus = status;
  renderRecordToggle();
  renderRecordCounter();
  await refreshAppState();
}

async function discoverDrones() {
  scanButton.disabled = true;
  networkMessage.textContent = "SCANNING";
  const result = await safeRequest("POST", "/api/drones/discover", {
    iface: ifaceSelect.value,
    rescan: true,
  });
  scanButton.disabled = false;
  if (!result) {
    networkMessage.textContent = "SCAN FAILED";
    return;
  }
  if (result.state) {
    state.drones = result.state.drones;
    state.selectedDroneId = state.drones.find((d) => d.connection?.iface === ifaceSelect.value)?.id ?? state.selectedDroneId;
  }
  const first = result.discovered?.[0];
  if (first) ssidInput.value = first.ssid;
  networkMessage.textContent = `FOUND ${result.discovered?.length ?? 0} DRONE AP`;
  state.treeSignature = "";
  renderTree();
  renderInspector();
}

async function connectSelectedWifi() {
  const ssid = ssidInput.value.trim();
  if (!ifaceSelect.value || !ssid) {
    networkMessage.textContent = "IFACE AND SSID REQUIRED";
    return;
  }
  connectButton.disabled = true;
  networkMessage.textContent = "CONNECTING";
  const result = await safeRequest("POST", "/api/wifi/connect", {
    iface: ifaceSelect.value,
    ssid,
    password: wifiPassword.value,
    confirmDisconnect: true,
  });
  connectButton.disabled = false;
  networkMessage.textContent = result?.ok ? "CONNECTED" : "CONNECT FAILED";
  await refreshNetwork();
}

async function reconnectWifi() {
  if (!ifaceSelect.value) return;
  reconnectButton.disabled = true;
  networkMessage.textContent = "RECONNECTING";
  const result = await safeRequest("POST", "/api/wifi/reconnect", {
    iface: ifaceSelect.value,
    password: wifiPassword.value,
  });
  reconnectButton.disabled = false;
  networkMessage.textContent = result?.ok ? "RECONNECTED" : "RECONNECT FAILED";
  await refreshNetwork();
}

async function refreshNetwork() {
  state.network = await safeRequest("GET", "/api/system/network");
  renderNetwork();
}

async function saveManualIoConfig() {
  const result = await safeRequest("POST", "/api/manual/config", {
    enabled: ioEnabled.checked,
    iface: manualIface.value.trim(),
    ip: manualIp.value.trim(),
    port: Number(manualPort.value),
    protocol: manualProtocol.value.trim(),
    bindDevice: manualBindDevice.checked,
  });
  if (!result) return;
  state.config = result;
  renderManualConfig();
  await refreshManualStatus();
}

async function savePolicy() {
  const flight = selectedFlight();
  if (!flight) return;
  const policy = {
    ...(flight.policy ?? {}),
    name: "Manual safety policy",
    version: 1,
    maxThrottle: Number(policyMaxThrottle.value),
    commandHz: Number(policyCommandHz.value),
    throttleSlewPerSecond: Number(policySlew.value),
    requireHeartbeat: policyHeartbeat.checked,
    singleActiveDrone: true,
  };
  const updated = await safeRequest("PATCH", `/api/flights/${flight.id}`, { policy });
  if (!updated) return;
  flight.policy = updated.policy;
  await saveManualPolicyConfig(policy);
  renderInspector();
}

async function saveManualPolicyConfig(policy) {
  const result = await safeRequest("POST", "/api/manual/config", {
    iface: manualIface.value.trim() || ifaceSelect.value,
    maxThrottle: policy.maxThrottle,
    commandHz: policy.commandHz,
    throttleSlewPerSecond: policy.throttleSlewPerSecond,
  });
  if (result) state.config = result;
}

async function importFrames() {
  const flight = selectedFlight();
  const source = importPath.value.trim();
  if (!flight || !source) return;
  const result = await safeRequest("POST", `/api/flights/${flight.id}/records`, {
    source,
    type: "frames",
    label: "Imported frame sequence",
    mime: "image/jpeg-sequence",
  });
  if (!result) return;
  importPath.value = "";
  await refreshAppState();
}

async function exportSelectedRecord(format) {
  const record = selectedFrameRecord();
  if (!record) return;
  const result = await safeRequest("POST", `/api/records/${record.id}/export`, { format, fps: 12 });
  if (!result) return;
  state.selectedRecordId = result.id;
  await refreshAppState();
}

async function revealRecord(recordId) {
  await safeRequest("POST", `/api/records/${recordId}/reveal`, {});
}

function startHeartbeat() {
  if (state.heartbeatTimer !== null) return;
  state.heartbeatTimer = window.setInterval(async () => {
    const status = await safeRequest("POST", "/api/manual/heartbeat", {});
    if (!status) return;
    const prev = state.manualStatus;
    state.manualStatus = status;
    if (
      !prev ||
      prev.state !== status.state ||
      prev.armed !== status.armed ||
      prev.faultReason !== status.faultReason
    ) {
      renderManualStatus();
    }
  }, 250);
}

function stopHeartbeat() {
  if (state.heartbeatTimer === null) return;
  window.clearInterval(state.heartbeatTimer);
  state.heartbeatTimer = null;
}

function selectedDrone() {
  return state.drones.find((d) => d.id === state.selectedDroneId);
}

function selectedFlight() {
  return selectedDrone()?.flights.find((f) => f.id === state.selectedFlightId);
}

function selectedFrameRecord() {
  const records = selectedFlight()?.records ?? [];
  return records.find((r) => r.id === state.selectedRecordId && r.type === "frames" && r.streamUrl)
    ?? records.find((r) => r.type === "frames" && r.streamUrl);
}

function element(tag, className = "") {
  const node = document.createElement(tag);
  if (className) node.className = className;
  return node;
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

async function loadState() {
  const result = await safeRequest("GET", "/api/state");
  return result;
}

async function safeRequest(method, path, body) {
  try {
    const result = await window.droneStation.request({ method, path, body });
    setService("READY");
    return result;
  } catch (error) {
    console.error(error);
    setService("ERR", true);
    return null;
  }
}

function setService(label, danger = false) {
  if (state.service === label) return;
  state.service = label;
  renderServiceStatus(danger);
}

function renderServiceStatus(danger) {
  serviceStatus.textContent = state.service;
  serviceStatus.classList.toggle("is-danger", Boolean(danger));
}

function absoluteServiceUrl(path) {
  return new URL(path, state.serviceUrl).toString();
}

function formatPolicy(policy) {
  if (!policy) return "—";
  if (typeof policy === "string") return policy;
  return policy.name || JSON.stringify(policy);
}

function formatValue(value) {
  if (typeof value === "number") {
    return Number.isInteger(value) ? value.toLocaleString() : value.toFixed(3);
  }
  return value;
}
