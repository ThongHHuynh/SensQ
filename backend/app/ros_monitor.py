import asyncio
import math
from datetime import datetime, timezone
from threading import Lock, Thread
from time import monotonic
from typing import Callable

from .config import CMD_VEL_TOPIC, JOINT_STATES_TOPIC, ODOM_TOPIC
from .database import save_snapshot
from .state import robot_state
from .websocket_manager import ws_manager


def yaw_from_quaternion(z: float, w: float) -> float:
    return math.degrees(math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z))


def occupancy_grid_to_payload(msg, max_cells: int = 260) -> dict:
    width = int(msg.info.width)
    height = int(msg.info.height)
    stride = max(1, math.ceil(max(width, height) / max_cells))
    sampled_width = math.ceil(width / stride)
    sampled_height = math.ceil(height / stride)
    source = list(msg.data)
    sampled = []

    for y in range(0, height, stride):
        for x in range(0, width, stride):
            occupied = 0
            free = 0
            unknown = 0
            for yy in range(y, min(y + stride, height)):
                row = yy * width
                for xx in range(x, min(x + stride, width)):
                    value = source[row + xx]
                    if value < 0:
                        unknown += 1
                    elif value >= 50:
                        occupied += 1
                    else:
                        free += 1

            if occupied > 0:
                sampled.append(100)
            elif free >= unknown:
                sampled.append(0)
            else:
                sampled.append(-1)

    origin = msg.info.origin
    return {
        "frame": msg.header.frame_id or "map",
        "width": sampled_width,
        "height": sampled_height,
        "resolution": round(float(msg.info.resolution) * stride, 4),
        "origin": {
            "x": round(float(origin.position.x), 3),
            "y": round(float(origin.position.y), 3),
            "yaw": round(yaw_from_quaternion(float(origin.orientation.z), float(origin.orientation.w)), 1),
        },
        "data": sampled,
        "updatedAt": datetime.now(timezone.utc).isoformat(),
        "status": f"Receiving /map ({width}x{height}, stride {stride})",
    }


class RosMonitor:
    def __init__(self, publish: Callable[[dict], None]) -> None:
        self._publish = publish
        self._thread: Thread | None = None
        self._lock = Lock()
        self._cmd_vel_publisher = None
        self._twist_type = None
        self._ros_available = False
        self._status_message = "ROS monitor has not started"

    def start(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import Twist
        except ImportError as exc:
            message = f"ROS command publisher disabled: {exc}"
            with self._lock:
                self._status_message = message

            snapshot = robot_state.update(
                {
                    "connection": {"lastHeartbeat": "ROS Python imports unavailable"},
                    "hardwareStatus": {"debug_message": message},
                }
            )
            robot_state.update_device("Web teleop", "offline", message, CMD_VEL_TOPIC)
            self._publish(snapshot)
            return

        def run() -> None:
            try:
                rclpy.init(args=None)
                node = rclpy.create_node("sensq_backend_monitor")
                cmd_vel_publisher = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
                with self._lock:
                    self._cmd_vel_publisher = cmd_vel_publisher
                    self._twist_type = Twist
                    self._ros_available = True
                    self._status_message = f"Publishing {CMD_VEL_TOPIC}"

                snapshot = robot_state.update_device("Web teleop", "online", f"Publishing {CMD_VEL_TOPIC}", CMD_VEL_TOPIC)
                self._publish(snapshot)
            except Exception as exc:
                message = f"ROS monitor failed to start: {exc}"
                with self._lock:
                    self._ros_available = False
                    self._status_message = message
                snapshot = robot_state.update_device("Web teleop", "offline", message, CMD_VEL_TOPIC)
                self._publish(snapshot)
                return

            try:
                from nav_msgs.msg import OccupancyGrid, Odometry
            except ImportError:
                OccupancyGrid = None
                Odometry = None

            try:
                from sensor_msgs.msg import JointState, LaserScan, Imu
            except ImportError:
                JointState = None
                LaserScan = None
                Imu = None

            def odom_cb(msg) -> None:
                now = datetime.now(timezone.utc).isoformat()
                pose = msg.pose.pose
                snapshot = robot_state.update(
                    {
                        "connection": {"lastHeartbeat": now, "launchState": "running"},
                        "pose": {
                            "frame": msg.header.frame_id or "odom",
                            "x": round(float(pose.position.x), 3),
                            "y": round(float(pose.position.y), 3),
                            "yaw": round(yaw_from_quaternion(float(pose.orientation.z), float(pose.orientation.w)), 1),
                        },
                        "navigation": {"localization": "Receiving odometry"},
                        "hardwareStatus": {
                            "are_motors_ready": True,
                            "debug_message": f"Receiving {ODOM_TOPIC}",
                            "updatedAt": now,
                        },
                    }
                )
                robot_state.update_device("Mobile base", "ready", f"Receiving odometry from {ODOM_TOPIC}", ODOM_TOPIC)
                self._publish(snapshot)

            def joint_cb(_) -> None:
                now = datetime.now(timezone.utc).isoformat()
                robot_state.update(
                    {
                        "connection": {"lastHeartbeat": now, "launchState": "running"},
                        "hardwareStatus": {
                            "are_motors_ready": True,
                            "debug_message": f"Receiving {JOINT_STATES_TOPIC}",
                            "updatedAt": now,
                        },
                    }
                )
                robot_state.update_device("Mobile base", "ready", f"Receiving wheel joint states from {JOINT_STATES_TOPIC}", JOINT_STATES_TOPIC)
                snapshot = robot_state.update_device("ros2_control", "online", f"Receiving {JOINT_STATES_TOPIC}")
                self._publish(snapshot)

            def scan_cb(_) -> None:
                snapshot = robot_state.update_device("Lidar", "online", "Receiving /scan")
                self._publish(snapshot)

            def imu_cb(_) -> None:
                snapshot = robot_state.update_device("IMU", "online", "Receiving /imu")
                self._publish(snapshot)

            def map_cb(msg) -> None:
                live_map = occupancy_grid_to_payload(msg)
                robot_state.update(
                    {
                        "liveMap": live_map,
                        "navigation": {"activeMap": "Live SLAM", "localization": "Map frame active"},
                    }
                )
                snapshot = robot_state.update_device("SLAM", "online", "Receiving /map from slam_toolbox", "/map")
                self._publish(snapshot)

            if Odometry is not None:
                node.create_subscription(Odometry, ODOM_TOPIC, odom_cb, 10)
            if OccupancyGrid is not None:
                node.create_subscription(OccupancyGrid, "/map", map_cb, 10)
            if JointState is not None:
                node.create_subscription(JointState, JOINT_STATES_TOPIC, joint_cb, 10)
            if LaserScan is not None:
                node.create_subscription(LaserScan, "/scan", scan_cb, 10)
            if Imu is not None:
                node.create_subscription(Imu, "/imu", imu_cb, 10)
            rclpy.spin(node)

        self._thread = Thread(target=run, daemon=True)
        self._thread.start()

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> tuple[bool, str]:
        with self._lock:
            publisher = self._cmd_vel_publisher
            twist_type = self._twist_type
            status_message = self._status_message

        if not self._ros_available or publisher is None or twist_type is None:
            return False, status_message

        twist = twist_type()
        twist.linear.x = max(-0.5, min(0.5, float(linear_x)))
        twist.angular.z = max(-1.5, min(1.5, float(angular_z)))
        publisher.publish(twist)
        return True, f"Published {CMD_VEL_TOPIC}"


def create_monitor(loop: asyncio.AbstractEventLoop) -> RosMonitor:
    snapshot_interval_seconds = 5.0
    last_snapshot_at = 0.0

    def publish(snapshot: dict) -> None:
        nonlocal last_snapshot_at

        asyncio.run_coroutine_threadsafe(ws_manager.broadcast(snapshot), loop)

        now = monotonic()
        if now - last_snapshot_at >= snapshot_interval_seconds:
            last_snapshot_at = now
            asyncio.run_coroutine_threadsafe(save_snapshot(snapshot), loop)

    return RosMonitor(publish)
