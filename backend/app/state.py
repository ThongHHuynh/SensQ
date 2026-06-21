from copy import deepcopy
from datetime import datetime, timezone


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def initial_snapshot() -> dict:
    return {
        "connection": {
            "mode": "backend",
            "backendUrl": "http://localhost:8000",
            "rosDomainId": "0",
            "lastHeartbeat": "never",
            "launchState": "stopped",
        },
        "hardwareStatus": {
            "temperature": None,
            "are_motors_ready": False,
            "debug_message": "Waiting for ROS hardware status",
            "updatedAt": None,
        },
        "battery": {
            "percent": None,
            "voltage": None,
            "state": "Unknown",
        },
        "pose": {
            "frame": "odom",
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
        },
        "navigation": {
            "state": "Idle",
            "activeMap": "Live SLAM",
            "localization": "Waiting",
        },
        "devices": [
            {"name": "Mobile base", "topic": "/hardware_status", "status": "offline", "detail": "Waiting for launch"},
            {"name": "ESP32", "topic": "/dev/ttyACM0", "status": "offline", "detail": "Serial link not confirmed"},
            {"name": "Lidar", "topic": "/scan", "status": "offline", "detail": "Waiting for scan"},
            {"name": "IMU", "topic": "/imu", "status": "offline", "detail": "Waiting for IMU"},
            {"name": "ros2_control", "topic": "/controller_manager", "status": "offline", "detail": "Controllers not confirmed"},
            {"name": "SLAM", "topic": "/map", "status": "offline", "detail": "Waiting for map"},
            {"name": "Web teleop", "topic": "/cmd_vel", "status": "offline", "detail": "Waiting for ROS publisher"},
        ],
        "maps": [
            {"id": "live", "name": "Live SLAM", "resolution": "From slam_toolbox", "updated": "Not synced"},
            {"id": "maze", "name": "Maze simulation", "resolution": "0.05 m/px", "updated": "From ros2_ws world reference"},
        ],
        "updatedAt": utc_now(),
    }


class RobotState:
    def __init__(self) -> None:
        self._snapshot = initial_snapshot()

    def get_snapshot(self) -> dict:
        return deepcopy(self._snapshot)

    def update(self, patch: dict) -> dict:
        self._deep_update(self._snapshot, patch)
        self._snapshot["updatedAt"] = utc_now()
        return self.get_snapshot()

    def update_device(self, name: str, status: str, detail: str | None = None, topic: str | None = None) -> dict:
        devices = self._snapshot["devices"]
        for device in devices:
            if device["name"] == name:
                device["status"] = status
                if detail is not None:
                    device["detail"] = detail
                if topic is not None:
                    device["topic"] = topic
                break
        else:
            devices.append({"name": name, "topic": topic or "", "status": status, "detail": detail or ""})

        self._snapshot["updatedAt"] = utc_now()
        return self.get_snapshot()

    def set_launch_state(self, launch_state: str) -> dict:
        return self.update({"connection": {"launchState": launch_state}})

    def _deep_update(self, target: dict, patch: dict) -> None:
        for key, value in patch.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                self._deep_update(target[key], value)
            else:
                target[key] = value


robot_state = RobotState()
