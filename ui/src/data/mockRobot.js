const mockMapData = Array.from({ length: 48 * 34 }, (_, index) => {
  const x = index % 48;
  const y = Math.floor(index / 48);
  const isWall = x === 4 || x === 43 || y === 4 || y === 29 || (x > 14 && x < 18 && y > 8 && y < 25);
  const isUnknown = x < 4 || x > 43 || y < 4 || y > 29;

  if (isWall) return 100;
  if (isUnknown) return -1;
  return 0;
});

export const robotSnapshot = {
  connection: {
    mode: "simulated",
    backendUrl: "http://localhost:8080",
    rosDomainId: "0",
    lastHeartbeat: "2s ago",
    launchState: "stopped"
  },
  hardwareStatus: {
    temperature: 31.8,
    are_motors_ready: true,
    debug_message: "Hardware interface standing by"
  },
  battery: {
    percent: 82,
    voltage: 23.7,
    state: "Discharging"
  },
  pose: {
    frame: "map",
    x: 1.42,
    y: -0.36,
    yaw: 18
  },
  navigation: {
    state: "Idle",
    activeMap: "Lab floor draft",
    localization: "Nominal"
  },
  liveMap: {
    frame: "map",
    width: 48,
    height: 34,
    resolution: 0.05,
    origin: { x: -1.2, y: -0.85, yaw: 0 },
    data: mockMapData,
    updatedAt: "Simulated",
    status: "Mock /map preview"
  },
  devices: [
    { name: "Mobile base", topic: "/joint_states", status: "ready", detail: "Wheel joint feedback" },
    { name: "ESP32", topic: "/dev/ttyACM0", status: "simulated", detail: "Serial link placeholder" },
    { name: "Lidar", topic: "/scan", status: "online", detail: "A1 profile expected" },
    { name: "IMU", topic: "/imu", status: "online", detail: "Publishing orientation" },
    { name: "ros2_control", topic: "/controller_manager", status: "simulated", detail: "Bridge pending" },
    { name: "SLAM", topic: "/map", status: "simulated", detail: "Bridge pending" },
    { name: "Web teleop", topic: "/cmd_vel", status: "simulated", detail: "Backend publisher placeholder" }
  ],
  maps: [
    { id: "lab", name: "Lab floor draft", resolution: "0.05 m/px", updated: "Not synced" },
    { id: "maze", name: "Maze simulation", resolution: "0.05 m/px", updated: "From ros2_ws world reference" }
  ]
};
