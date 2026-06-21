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
  devices: [
    { name: "Mobile base", topic: "/hardware_status", status: "ready", detail: "Motors ready" },
    { name: "ESP32", topic: "/dev/ttyACM0", status: "simulated", detail: "Serial link placeholder" },
    { name: "Lidar", topic: "/scan", status: "online", detail: "A1 profile expected" },
    { name: "IMU", topic: "/imu", status: "online", detail: "Publishing orientation" },
    { name: "ros2_control", topic: "/controller_manager", status: "simulated", detail: "Bridge pending" },
    { name: "SLAM", topic: "/map", status: "simulated", detail: "Bridge pending" }
  ],
  maps: [
    { id: "lab", name: "Lab floor draft", resolution: "0.05 m/px", updated: "Not synced" },
    { id: "maze", name: "Maze simulation", resolution: "0.05 m/px", updated: "From ros2_ws world reference" }
  ]
};
