import asyncio
import math
from datetime import datetime, timezone
from threading import Lock, Thread
from typing import Callable

from .config import CMD_VEL_TOPIC, HARDWARE_STATUS_TOPIC, JOINT_STATES_TOPIC, ODOM_TOPIC, SERIAL_PORT
from .database import save_snapshot
from .state import robot_state
from .websocket_manager import ws_manager


def yaw_from_quaternion(z: float, w: float) -> float:
    return math.degrees(math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z))


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
                from my_robot_interfaces.msg import HardwareStatus
            except ImportError as exc:
                snapshot = robot_state.update(
                    {"hardwareStatus": {"debug_message": f"HardwareStatus subscription unavailable: {exc}"}}
                )
                self._publish(snapshot)
                HardwareStatus = None

            try:
                from nav_msgs.msg import Odometry
            except ImportError:
                Odometry = None

            try:
                from sensor_msgs.msg import JointState, LaserScan, Imu
            except ImportError:
                JointState = None
                LaserScan = None
                Imu = None

            def hardware_cb(msg) -> None:
                now = datetime.now(timezone.utc).isoformat()
                snapshot = robot_state.update(
                    {
                        "connection": {"lastHeartbeat": now, "launchState": "running"},
                        "hardwareStatus": {
                            "temperature": float(msg.temperature),
                            "are_motors_ready": bool(msg.are_motors_ready),
                            "debug_message": msg.debug_message,
                            "updatedAt": now,
                        },
                    }
                )
                robot_state.update_device("Mobile base", "ready" if msg.are_motors_ready else "warning", msg.debug_message)
                robot_state.update_device("ESP32", "online", "HardwareStatus received from serial-connected base", SERIAL_PORT)
                self._publish(snapshot)

            def odom_cb(msg) -> None:
                pose = msg.pose.pose
                snapshot = robot_state.update(
                    {
                        "pose": {
                            "frame": msg.header.frame_id or "odom",
                            "x": round(float(pose.position.x), 3),
                            "y": round(float(pose.position.y), 3),
                            "yaw": round(yaw_from_quaternion(float(pose.orientation.z), float(pose.orientation.w)), 1),
                        },
                        "navigation": {"localization": "Receiving odometry"},
                    }
                )
                self._publish(snapshot)

            def joint_cb(_) -> None:
                snapshot = robot_state.update_device("ros2_control", "online", f"Receiving {JOINT_STATES_TOPIC}")
                self._publish(snapshot)

            def scan_cb(_) -> None:
                snapshot = robot_state.update_device("Lidar", "online", "Receiving /scan")
                self._publish(snapshot)

            def imu_cb(_) -> None:
                snapshot = robot_state.update_device("IMU", "online", "Receiving /imu")
                self._publish(snapshot)

            if HardwareStatus is not None:
                node.create_subscription(HardwareStatus, HARDWARE_STATUS_TOPIC, hardware_cb, 10)
            if Odometry is not None:
                node.create_subscription(Odometry, ODOM_TOPIC, odom_cb, 10)
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
    def publish(snapshot: dict) -> None:
        asyncio.run_coroutine_threadsafe(ws_manager.broadcast(snapshot), loop)
        asyncio.run_coroutine_threadsafe(save_snapshot(snapshot), loop)

    return RosMonitor(publish)
