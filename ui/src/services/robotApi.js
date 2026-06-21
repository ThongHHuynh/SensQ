import { robotSnapshot } from "../data/mockRobot.js";

const API_BASE_URL = import.meta.env.VITE_API_BASE_URL ?? "http://localhost:8000";
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

export function createRobotStateSocket() {
  return new WebSocket(`${WS_BASE_URL}/ws/robot-state`);
}
