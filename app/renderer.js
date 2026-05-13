const state = {
  drones: [],
  selectedDroneId: "",
  selectedFlightId: "",
  mainView: "forward",
  lhsCollapsed: false,
  rhsCollapsed: false,
  mode: "review",
  serviceUrl: "",
  manualStatus: null,
  heartbeatTimer: null,
  refreshTimer: null,
};

const workspace = document.querySelector(".workspace");
const droneTree = document.getElementById("droneTree");
const droneCount = document.getElementById("droneCount");
const metadataList = document.getElementById("metadataList");
const metricsGrid = document.getElementById("metricsGrid");
const recordsList = document.getElementById("recordsList");
const flightState = document.getElementById("flightState");
const manualPanel = document.getElementById("manualPanel");
const throttle = document.getElementById("throttle");
const throttleValue = document.getElementById("throttleValue");
const forwardStream = document.getElementById("forwardStream");
const forwardEmpty = document.getElementById("forwardEmpty");
const serviceStatus = document.getElementById("serviceStatus");
const manualState = document.getElementById("manualState");
const manualMessage = document.getElementById("manualMessage");
const armButton = document.getElementById("armButton");
const disarmButton = document.getElementById("disarmButton");
const stopButton = document.getElementById("stopButton");

init();

async function init() {
  state.serviceUrl = await window.droneStation.serviceUrl();
  const initialState = await loadState();
  state.drones = initialState.drones;
  state.selectedDroneId = state.drones[0]?.id ?? "";
  state.selectedFlightId = state.drones[0]?.flights[0]?.id ?? "";
  await refreshManualStatus();
  render();
  wireToolbar();
  wireModeSelector();
  wireSimulation();
  wireControls();
  state.refreshTimer = window.setInterval(refreshAppState, 5000);
}

function wireToolbar() {
  document.getElementById("lhsToggle").addEventListener("click", () => {
    state.lhsCollapsed = !state.lhsCollapsed;
    applyLayout();
  });

  document.getElementById("rhsToggle").addEventListener("click", () => {
    state.rhsCollapsed = !state.rhsCollapsed;
    applyLayout();
  });

  document.querySelectorAll(".segment").forEach((button) => {
    button.addEventListener("click", () => {
      state.mainView = button.dataset.view;
      renderMainView();
    });
  });

  document.getElementById("newFlight").addEventListener("click", () => {
    createDraftFlight();
  });
}

function wireModeSelector() {
  document.querySelectorAll(".mode-option").forEach((button) => {
    button.addEventListener("click", async () => {
      if (state.mode === "manual" && button.dataset.mode !== "manual") {
        await safeApiPost("/api/manual/disarm", {});
        await refreshManualStatus();
      }
      state.mode = button.dataset.mode;
      await persistSelectedFlightMode(state.mode);
      renderMode();
    });
  });
}

function wireSimulation() {
  document.getElementById("trajectoryToggle").addEventListener("change", (event) => {
    document.querySelector(".simulation-stage").classList.toggle("hide-trajectory", !event.target.checked);
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
      await safeApiPost("/api/manual/clear-fault", {});
    }
    const status = await safeApiPost("/api/manual/arm", {});
    if (status) updateManualStatus(status);
  });
  disarmButton.addEventListener("click", async () => {
    const status = await safeApiPost("/api/manual/disarm", {});
    if (status) updateManualStatus(status);
  });
  stopButton.addEventListener("click", emergencyStop);
}

window.addEventListener("beforeunload", () => {
  if (state.heartbeatTimer !== null) {
    window.clearInterval(state.heartbeatTimer);
  }
  if (state.refreshTimer !== null) {
    window.clearInterval(state.refreshTimer);
  }
});

async function emergencyStop() {
  const status = await safeApiPost("/api/manual/stop", {});
  if (status) {
    throttle.value = "0";
    throttleValue.textContent = "0";
    updateManualStatus(status);
  }
}

async function sendManualAxes(axes) {
  if (state.mode !== "manual") return;
  const status = await safeApiPost("/api/manual/axes", axes);
  if (status) updateManualStatus(status);
}

function controlAxes(control) {
  const center = 128;
  const step = 28;
  switch (control) {
    case "pitch-up":
      return { pitch: center + step };
    case "pitch-down":
      return { pitch: center - step };
    case "roll-left":
      return { roll: center - step };
    case "roll-right":
      return { roll: center + step };
    case "yaw-left":
      return { yaw: center - step };
    case "yaw-right":
      return { yaw: center + step };
    default:
      return {};
  }
}

async function refreshManualStatus() {
  const status = await safeApiGet("/api/manual/status");
  if (status) updateManualStatus(status);
}

function updateManualStatus(status) {
  state.manualStatus = status;
  renderManualStatus();
}

function renderManualStatus() {
  const status = state.manualStatus;
  if (!status) return;
  manualState.textContent = status.state;
  manualState.classList.toggle("is-danger", status.state === "faulted");
  manualState.classList.toggle("is-armed", status.armed);
  manualMessage.textContent = status.faultReason
    ? `Fault: ${status.faultReason}`
    : status.stopReason
      ? `Stopping: ${status.stopReason}`
      : manualStatusText(status);
  armButton.textContent = status.state === "faulted" ? "Clear + Arm" : "Arm";
  armButton.disabled = status.armed;
  disarmButton.disabled = !status.armed;
  throttle.disabled = !status.armed;
  document.querySelectorAll("[data-control]").forEach((button) => {
    if (button.dataset.control !== "stop") {
      button.disabled = !status.armed;
    }
  });
}

function manualStatusText(status) {
  const transport = status.transport;
  const stateText = status.armed ? "Manual control is armed. Heartbeat active." : "Manual control is disarmed.";
  if (!transport?.enabled) {
    return `${stateText} Drone IO disabled.`;
  }
  if (transport.lastError) {
    return `${stateText} Transport error: ${transport.lastError}`;
  }
  return `${stateText} Transport ${transport.connected ? "connected" : "ready"}: ${transport.target}.`;
}

function render() {
  applyLayout();
  renderTree();
  renderMainView();
  renderInspector();
  renderStream();
}

function applyLayout() {
  workspace.classList.toggle("lhs-collapsed", state.lhsCollapsed);
  workspace.classList.toggle("rhs-collapsed", state.rhsCollapsed);
}

function renderTree() {
  droneCount.textContent = state.drones.length;
  droneTree.replaceChildren();

  state.drones.forEach((drone) => {
    const group = element("div", "tree-group is-expanded");
    const droneButton = element("button", "drone-row");
    droneButton.innerHTML = `
      <span class="chevron">▾</span>
      <span class="tree-name">${escapeHtml(drone.name)}<span class="tree-subtitle">${escapeHtml(drone.model)} · ${escapeHtml(drone.lastSeen)}</span></span>
      <span class="status-dot ${escapeHtml(drone.status)}"></span>
    `;
    droneButton.addEventListener("click", () => {
      group.classList.toggle("is-expanded");
      state.selectedDroneId = drone.id;
      if (!state.selectedFlightId && drone.flights[0]) {
        state.selectedFlightId = drone.flights[0].id;
      }
      renderInspector();
    });

    const children = element("div", "children");
    if (drone.flights.length === 0) {
      const empty = element("div", "flight-row");
      empty.innerHTML = `<span></span><span class="tree-name">No flights<span class="tree-subtitle">ready for first record</span></span><span></span>`;
      children.append(empty);
    }

    drone.flights.forEach((flight) => {
      const flightButton = element("button", `flight-row ${flight.id === state.selectedFlightId ? "is-active" : ""}`);
      flightButton.innerHTML = `
        <span></span>
        <span class="tree-name">${escapeHtml(flight.name)}<span class="tree-subtitle">${escapeHtml(flight.duration)} · ${escapeHtml(flight.mode)}</span></span>
        <span></span>
      `;
      flightButton.addEventListener("click", () => {
        state.selectedDroneId = drone.id;
        state.selectedFlightId = flight.id;
        state.mode = flight.mode ?? "review";
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
  document.querySelectorAll(".segment").forEach((button) => {
    button.classList.toggle("is-active", button.dataset.view === state.mainView);
  });

  document.querySelectorAll(".view-panel").forEach((panel) => {
    panel.classList.remove("is-active");
  });
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
    return;
  }

  if (flight) {
    state.mode = flight.mode ?? state.mode;
  }
  flightState.textContent = state.mode;
  renderMode();

  const metadata = {
    Drone: drone.name,
    SSID: drone.connection.ssid,
    Interface: drone.connection.iface,
    IP: drone.connection.ip,
    Control: drone.connection.control,
    Camera: drone.connection.camera,
    Policy: formatPolicy(flight?.policy),
    Started: flight?.startedAt ?? "not started",
    ...(flight?.metadata ?? {}),
  };
  renderKeyValue(metadataList, metadata);

  renderMetrics(flight?.metrics ?? {});
  renderRecords(flight?.records ?? []);
  renderStream();
}

function renderMode() {
  document.querySelectorAll(".mode-option").forEach((button) => {
    button.classList.toggle("is-active", button.dataset.mode === state.mode);
  });
  flightState.textContent = state.mode;
  manualPanel.classList.toggle("is-hidden", state.mode !== "manual");
  if (state.mode === "manual") {
    startHeartbeat();
  } else {
    stopHeartbeat();
  }
  renderManualStatus();
}

function renderMetrics(metrics) {
  metricsGrid.replaceChildren();
  const entries = [
    ["Frames", metrics.frames],
    ["Packets", metrics.packets],
    ["Bytes", metrics.bytes],
    ["Resolution", metrics.resolution],
    ["Temporal MAE", metrics.temporalMae],
    ["Smoothed", metrics.smoothedTemporalMae],
  ].filter(([, value]) => value !== undefined && value !== null);

  entries.forEach(([label, value]) => {
    const item = element("div", "metric");
    item.innerHTML = `<span>${escapeHtml(label)}</span><strong>${escapeHtml(formatValue(value))}</strong>`;
    metricsGrid.append(item);
  });
}

function renderRecords(records) {
  recordsList.replaceChildren();
  records.forEach((record) => {
    const item = element("div", "record");
    item.innerHTML = `<strong>${escapeHtml(record.label)}</strong><code>${escapeHtml(record.path ?? record.blobKey ?? "not imported")}</code>`;
    recordsList.append(item);
  });
}

function renderKeyValue(parent, values) {
  parent.replaceChildren();
  Object.entries(values).forEach(([key, value]) => {
    const dt = element("dt");
    dt.textContent = key;
    const dd = element("dd");
    dd.textContent = String(value);
    parent.append(dt, dd);
  });
}

async function createDraftFlight() {
  const drone = selectedDrone();
  if (!drone) return;

  const now = new Date();
  const created = await apiPost("/api/flights", {
    droneId: drone.id,
    name: `Draft flight ${now.toLocaleTimeString()}`,
  });
  const refreshed = await loadState();
  state.drones = refreshed.drones;
  state.selectedFlightId = created.id;
  state.mode = "manual";
  renderTree();
  renderInspector();
}

function renderStream() {
  const flight = selectedFlight();
  const framesRecord = flight?.records.find((record) => record.type === "frames" && record.streamUrl);
  if (framesRecord && state.mainView === "forward") {
    const nextUrl = absoluteServiceUrl(`${framesRecord.streamUrl}?fps=12`);
    if (forwardStream.src !== nextUrl) {
      forwardStream.src = nextUrl;
    }
    forwardStream.classList.remove("is-hidden");
    forwardEmpty.classList.add("is-hidden");
    return;
  }
  if (forwardStream.hasAttribute("src")) {
    forwardStream.removeAttribute("src");
  }
  forwardStream.classList.add("is-hidden");
  forwardEmpty.classList.remove("is-hidden");
}

async function persistSelectedFlightMode(mode) {
  const flight = selectedFlight();
  if (!flight) return;
  const updated = await safeApiPatch(`/api/flights/${flight.id}`, { mode });
  if (!updated) return;
  flight.mode = updated.mode;
}

async function refreshAppState() {
  const refreshed = await safeApiGet("/api/state");
  if (!refreshed) return;
  const selectedDroneId = state.selectedDroneId;
  const selectedFlightId = state.selectedFlightId;
  state.drones = refreshed.drones;
  state.selectedDroneId = state.drones.find((drone) => drone.id === selectedDroneId)?.id ?? state.drones[0]?.id ?? "";
  state.selectedFlightId =
    selectedFlight()?.id ??
    state.drones.find((drone) => drone.id === state.selectedDroneId)?.flights[0]?.id ??
    selectedFlightId;
  renderTree();
  renderInspector();
}

function startHeartbeat() {
  if (state.heartbeatTimer !== null) return;
  state.heartbeatTimer = window.setInterval(async () => {
    const status = await safeApiPost("/api/manual/heartbeat", {});
    if (status) updateManualStatus(status);
  }, 250);
}

function stopHeartbeat() {
  if (state.heartbeatTimer === null) return;
  window.clearInterval(state.heartbeatTimer);
  state.heartbeatTimer = null;
}

function selectedDrone() {
  return state.drones.find((drone) => drone.id === state.selectedDroneId);
}

function selectedFlight() {
  const drone = selectedDrone();
  return drone?.flights.find((flight) => flight.id === state.selectedFlightId);
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

async function apiGet(path) {
  return window.droneStation.request({ method: "GET", path });
}

async function apiPost(path, body) {
  return window.droneStation.request({ method: "POST", path, body });
}

async function apiPatch(path, body) {
  return window.droneStation.request({ method: "PATCH", path, body });
}

async function loadState() {
  const loaded = await apiGet("/api/state");
  serviceStatus.textContent = "Service online";
  serviceStatus.classList.remove("is-danger");
  return loaded;
}

async function safeApiGet(path) {
  try {
    const result = await apiGet(path);
    serviceStatus.textContent = "Service online";
    serviceStatus.classList.remove("is-danger");
    return result;
  } catch (error) {
    serviceStatus.textContent = "Service error";
    serviceStatus.classList.add("is-danger");
    console.error(error);
    return null;
  }
}

async function safeApiPost(path, body) {
  try {
    const result = await apiPost(path, body);
    serviceStatus.textContent = "Service online";
    serviceStatus.classList.remove("is-danger");
    return result;
  } catch (error) {
    serviceStatus.textContent = "Service error";
    serviceStatus.classList.add("is-danger");
    console.error(error);
    return null;
  }
}

async function safeApiPatch(path, body) {
  try {
    const result = await apiPatch(path, body);
    serviceStatus.textContent = "Service online";
    serviceStatus.classList.remove("is-danger");
    return result;
  } catch (error) {
    serviceStatus.textContent = "Service error";
    serviceStatus.classList.add("is-danger");
    console.error(error);
    return null;
  }
}

function absoluteServiceUrl(path) {
  return new URL(path, state.serviceUrl).toString();
}

function formatPolicy(policy) {
  if (!policy) return "No flight selected";
  if (typeof policy === "string") return policy;
  return policy.name || JSON.stringify(policy);
}

function formatValue(value) {
  if (typeof value === "number") {
    return Number.isInteger(value) ? value.toLocaleString() : value.toFixed(3);
  }
  return value;
}
