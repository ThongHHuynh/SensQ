const launchSelect = document.getElementById("launchSelect");
const launchDescription = document.getElementById("launchDescription");
const statusBadge = document.getElementById("statusBadge");
const currentLaunch = document.getElementById("currentLaunch");
const uptime = document.getElementById("uptime");
const returnCode = document.getElementById("returnCode");
const logOutput = document.getElementById("logOutput");
const startButton = document.getElementById("startButton");
const stopButton = document.getElementById("stopButton");
const refreshButton = document.getElementById("refreshButton");

let cachedLaunchOptions = [];

function optionByKey(key) {
  return cachedLaunchOptions.find((option) => option.key === key);
}

function renderLaunchOptions(options) {
  cachedLaunchOptions = options;
  const selectedKey = launchSelect.value;
  launchSelect.innerHTML = "";

  for (const option of options) {
    const element = document.createElement("option");
    element.value = option.key;
    element.textContent = option.label;
    launchSelect.appendChild(element);
  }

  if (selectedKey && optionByKey(selectedKey)) {
    launchSelect.value = selectedKey;
  }

  updateLaunchDescription();
}

function updateLaunchDescription() {
  const selected = optionByKey(launchSelect.value);
  launchDescription.textContent = selected ? selected.description : "";
}

function setBadge(running) {
  statusBadge.textContent = running ? "Running" : "Idle";
  statusBadge.className = `status-badge ${running ? "running" : "idle"}`;
}

function renderStatus(status) {
  if (status.launch_options && status.launch_options.length > 0) {
    renderLaunchOptions(status.launch_options);
  }

  setBadge(Boolean(status.running));
  currentLaunch.textContent = status.current_launch ? status.current_launch.label : "None";
  uptime.textContent = status.uptime_seconds ? `${status.uptime_seconds}s` : "-";
  returnCode.textContent = status.return_code ?? "-";
  logOutput.textContent = (status.log_lines || []).join("\n") || "No output yet.";

  startButton.disabled = Boolean(status.running);
  stopButton.disabled = !status.running;
}

async function fetchStatus() {
  const response = await fetch("/api/status");
  const data = await response.json();
  renderStatus(data);
}

async function startLaunch() {
  const response = await fetch("/api/start", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ launch_key: launchSelect.value }),
  });

  const data = await response.json();
  if (!response.ok) {
    alert(data.error || "Failed to start launch.");
    return;
  }
  renderStatus(data);
}

async function stopLaunch() {
  const response = await fetch("/api/stop", { method: "POST" });
  const data = await response.json();
  renderStatus(data);
}

launchSelect.addEventListener("change", updateLaunchDescription);
startButton.addEventListener("click", startLaunch);
stopButton.addEventListener("click", stopLaunch);
refreshButton.addEventListener("click", fetchStatus);

fetchStatus();
setInterval(fetchStatus, 2000);
