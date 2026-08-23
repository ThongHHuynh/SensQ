import { robotSnapshot } from "../data/mockRobot.js";

export const API_BASE_URL = import.meta.env.VITE_API_BASE_URL ?? "http://localhost:8000";
const WS_BASE_URL = import.meta.env.VITE_WS_BASE_URL ?? API_BASE_URL.replace(/^http/, "ws");

export function getRobotSnapshot() {
  return robotSnapshot;
}

export async function fetchRobotSnapshot() {
  const response = await fetch(`${API_BASE_URL}/api/robot/snapshot`);

  if (!response.ok) {
    throw new Error(`Snapshot request failed: ${response.status}`);
  }

  return response.json();
}

export async function startMobileBase() {
  const response = await fetch(`${API_BASE_URL}/api/robot/launch/mobile-base`, {
    method: "POST"
  });

  if (!response.ok) {
    throw new Error(`Launch request failed: ${response.status}`);
  }

  return response.json();
}

export async function stopRobot() {
  const response = await fetch(`${API_BASE_URL}/api/robot/stop`, {
    method: "POST"
  });

  if (!response.ok) {
    throw new Error(`Stop request failed: ${response.status}`);
  }

  return response.json();
}

export async function startTeleop() {
  const response = await fetch(`${API_BASE_URL}/api/teleop/start`, {
    method: "POST"
  });

  if (!response.ok) {
    throw new Error(`Teleop start failed: ${response.status}`);
  }

  return response.json();
}

export async function stopTeleop() {
  const response = await fetch(`${API_BASE_URL}/api/teleop/stop`, {
    method: "POST"
  });

  if (!response.ok) {
    throw new Error(`Teleop stop failed: ${response.status}`);
  }

  return response.json();
}

export async function sendTeleopCommand({ linear_x = 0, angular_z = 0 }) {
  const response = await fetch(`${API_BASE_URL}/api/teleop/cmd_vel`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ linear_x, angular_z })
  });

  if (!response.ok) {
    throw new Error(`Teleop command failed: ${response.status}`);
  }

  return response.json();
}

async function dockingResponse(response, fallback) {
  const payload = await response.json();
  if (!response.ok) {
    throw new Error(payload.detail || fallback);
  }
  return payload;
}

async function apiResponse(response, fallback) {
  let payload = {};
  try {
    payload = await response.json();
  } catch {
    payload = {};
  }
  if (!response.ok) {
    throw new Error(payload.detail || payload.message || fallback);
  }
  return payload;
}

export async function fetchDockingConfig() {
  const response = await fetch(`${API_BASE_URL}/api/docking/config`);
  return dockingResponse(response, `Docking configuration failed: ${response.status}`);
}

export async function startDocking(goal) {
  const response = await fetch(`${API_BASE_URL}/api/docking/start`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(goal)
  });
  return dockingResponse(response, `Docking start failed: ${response.status}`);
}

export async function cancelDocking() {
  const response = await fetch(`${API_BASE_URL}/api/docking/cancel`, {
    method: "POST"
  });
  return dockingResponse(response, `Docking cancellation failed: ${response.status}`);
}

export async function undockRobot(dockId) {
  const response = await fetch(`${API_BASE_URL}/api/docking/undock`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ dock_id: dockId })
  });
  return dockingResponse(response, `Undock start failed: ${response.status}`);
}

export async function cancelUndock() {
  const response = await fetch(`${API_BASE_URL}/api/docking/undock/cancel`, {
    method: "POST"
  });
  return dockingResponse(response, `Undock cancellation failed: ${response.status}`);
}

export async function saveDockStation(station) {
  const response = await fetch(`${API_BASE_URL}/api/docking/stations`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(station)
  });
  return apiResponse(response, `Dock station save failed: ${response.status}`);
}

export async function startMapping() {
  const response = await fetch(`${API_BASE_URL}/api/mapping/start`, {
    method: "POST"
  });

  if (!response.ok) {
    throw new Error(`Mapping start failed: ${response.status}`);
  }

  return response.json();
}

export async function stopMapping() {
  const response = await fetch(`${API_BASE_URL}/api/mapping/stop`, {
    method: "POST"
  });

  if (!response.ok) {
    throw new Error(`Mapping stop failed: ${response.status}`);
  }

  return response.json();
}

export async function resetMapping() {
  const response = await fetch(`${API_BASE_URL}/api/mapping/reset`, {
    method: "POST"
  });

  if (!response.ok) {
    throw new Error(`Mapping reset failed: ${response.status}`);
  }

  return response.json();
}

export async function saveMap(name) {
  const response = await fetch(`${API_BASE_URL}/api/mapping/save`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name })
  });

  if (!response.ok) {
    throw new Error(`Map save failed: ${response.status}`);
  }

  return response.json();
}

export async function selectMap(mapId) {
  const response = await fetch(`${API_BASE_URL}/api/maps/${encodeURIComponent(mapId)}/select`, {
    method: "POST"
  });

  return apiResponse(response, `Map select failed: ${response.status}`);
}

export async function renameMap(mapId, name) {
  const response = await fetch(`${API_BASE_URL}/api/maps/${encodeURIComponent(mapId)}`, {
    method: "PATCH",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name })
  });

  if (!response.ok) {
    throw new Error(`Map rename failed: ${response.status}`);
  }

  return response.json();
}

export async function setInitialPose(pose) {
  const response = await fetch(`${API_BASE_URL}/api/navigation/initial-pose`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(pose)
  });
  return apiResponse(response, `Initial pose failed: ${response.status}`);
}

export async function sendNavigationGoal(pose) {
  const response = await fetch(`${API_BASE_URL}/api/navigation/goals`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(pose)
  });
  return apiResponse(response, `Navigation goal failed: ${response.status}`);
}

export async function cancelNavigationGoal() {
  const response = await fetch(`${API_BASE_URL}/api/navigation/cancel`, { method: "POST" });
  return apiResponse(response, `Navigation cancellation failed: ${response.status}`);
}

export async function prepareCoverage(useSimTime = false) {
  const response = await fetch(`${API_BASE_URL}/api/coverage/prepare`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ use_sim_time: useSimTime })
  });
  return apiResponse(response, `Coverage startup failed: ${response.status}`);
}

export async function generateCoverage() {
  const response = await fetch(`${API_BASE_URL}/api/coverage/generate`, { method: "POST" });
  return apiResponse(response, `Coverage generation failed: ${response.status}`);
}

export async function executeCoverage() {
  const response = await fetch(`${API_BASE_URL}/api/coverage/execute`, { method: "POST" });
  return apiResponse(response, `Coverage execution failed: ${response.status}`);
}

export async function cancelCoverage() {
  const response = await fetch(`${API_BASE_URL}/api/coverage/cancel`, { method: "POST" });
  return apiResponse(response, `Coverage cancellation failed: ${response.status}`);
}

export async function fetchCameraSnapshot() {
  const response = await fetch(`${API_BASE_URL}/api/camera/snapshot`);
  if (!response.ok) {
    let detail = null;
    try {
      const payload = await response.json();
      detail = payload.detail || payload.message;
    } catch {
      detail = null;
    }
    throw new Error(detail || `Snapshot request failed: ${response.status}`);
  }
  return response.blob();
}

export async function fetchSettings() {
  const response = await fetch(`${API_BASE_URL}/api/settings`);
  return apiResponse(response, `Settings request failed: ${response.status}`);
}

export async function updateSettings(values) {
  const response = await fetch(`${API_BASE_URL}/api/settings`, {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(values)
  });
  return apiResponse(response, `Settings update failed: ${response.status}`);
}

export async function startMission(missionType, dockId, autoDock) {
  const response = await fetch(`${API_BASE_URL}/api/mission/start`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      mission_type: missionType,
      dock_id: dockId,
      auto_dock_on_complete: autoDock
    })
  });
  return apiResponse(response, `Mission start failed: ${response.status}`);
}

export async function cancelMission() {
  const response = await fetch(`${API_BASE_URL}/api/mission/cancel`, { method: "POST" });
  return apiResponse(response, `Mission cancellation failed: ${response.status}`);
}

export async function getMissionHistory() {
  const response = await fetch(`${API_BASE_URL}/api/mission/history`);
  return apiResponse(response, `Mission history request failed: ${response.status}`);
}

export function createRobotStateSocket() {
  return new WebSocket(`${WS_BASE_URL}/ws/robot-state`);
}
