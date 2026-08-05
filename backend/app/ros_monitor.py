import asyncio
import math
from datetime import datetime, timezone
from threading import Lock, Thread
from time import monotonic
from typing import Callable

from .config import CMD_VEL_TOPIC, JOINT_STATES_TOPIC, ODOM_TOPIC
from .database import save_snapshot
from .state import robot_state, utc_now
from .websocket_manager import ws_manager


def yaw_from_quaternion(z: float, w: float) -> float:
    return math.degrees(math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z))


def yaw_from_full_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


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
    def __init__(self, publish: Callable[[dict], None], dock_action_name: str) -> None:
        self._publish = publish
        self._thread: Thread | None = None
        self._lock = Lock()
        self._cmd_vel_publisher = None
        self._twist_type = None
        self._ros_available = False
        self._status_message = "ROS monitor has not started"
        self._dock_action_name = dock_action_name
        self._dock_client = None
        self._dock_type = None
        self._dock_goal_handle = None
        self._dock_goal_pending = False
        self._dock_cancel_requested = False
        self._dock_feedback_states = {}
        self._dock_goal_statuses = {}

    def start(self) -> None:
        try:
            import rclpy
            from action_msgs.msg import GoalStatus
            from geometry_msgs.msg import Twist
            from my_robot_docking_msgs.action import Dock
            from rclpy.action import ActionClient
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
                dock_client = ActionClient(
                    node,
                    Dock,
                    self._dock_action_name,
                )
                with self._lock:
                    self._cmd_vel_publisher = cmd_vel_publisher
                    self._twist_type = Twist
                    self._ros_available = True
                    self._status_message = f"Publishing {CMD_VEL_TOPIC}"
                    self._dock_client = dock_client
                    self._dock_type = Dock
                    self._dock_feedback_states = {
                        Dock.Feedback.IDLE: "IDLE",
                        Dock.Feedback.NAVIGATING: "NAVIGATING",
                        Dock.Feedback.SEARCHING: "SEARCHING",
                        Dock.Feedback.APPROACHING: "APPROACHING",
                        Dock.Feedback.ALIGNING: "ALIGNING",
                        Dock.Feedback.VERIFYING: "VERIFYING",
                        Dock.Feedback.RETRYING: "RETRYING",
                    }
                    self._dock_goal_statuses = {
                        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
                        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
                        GoalStatus.STATUS_EXECUTING: "EXECUTING",
                        GoalStatus.STATUS_CANCELING: "CANCELING",
                        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                        GoalStatus.STATUS_CANCELED: "CANCELED",
                        GoalStatus.STATUS_ABORTED: "ABORTED",
                    }

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

            try:
                import tf2_ros
                tf_buffer = tf2_ros.Buffer()
                tf2_ros.TransformListener(tf_buffer, node)
            except Exception:
                tf_buffer = None

            def update_map_pose() -> None:
                if tf_buffer is None:
                    return
                try:
                    transform = tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                except Exception:
                    return

                translation = transform.transform.translation
                rotation = transform.transform.rotation
                snapshot = robot_state.update(
                    {
                        "pose": {
                            "frame": "map",
                            "x": round(float(translation.x), 3),
                            "y": round(float(translation.y), 3),
                            "yaw": round(
                                yaw_from_full_quaternion(
                                    float(rotation.x),
                                    float(rotation.y),
                                    float(rotation.z),
                                    float(rotation.w),
                                ),
                                1,
                            ),
                        },
                        "navigation": {"localization": "Receiving map TF"},
                    }
                )
                self._publish(snapshot)

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
            if tf_buffer is not None:
                node.create_timer(0.5, update_map_pose)
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

    def start_docking(self, request: dict) -> tuple[bool, str]:
        with self._lock:
            client = self._dock_client
            dock_type = self._dock_type
            active = self._dock_goal_pending or self._dock_goal_handle is not None

        if active:
            return False, "A docking goal is already active"
        if not self._ros_available or client is None or dock_type is None:
            return False, "ROS docking client is not initialized"
        if not client.server_is_ready():
            return False, f"Dock action {self._dock_action_name} is unavailable"

        goal = dock_type.Goal()
        goal.dock_id = request["dock_id"]
        goal.navigate_to_staging_pose = request["navigate_to_staging_pose"]
        goal.use_offset_override = request["use_offset_override"]
        goal.final_distance = request["final_distance"]
        goal.lateral_offset = request["lateral_offset"]
        goal.yaw_offset = request["yaw_offset"]

        with self._lock:
            self._dock_goal_pending = True
            self._dock_cancel_requested = False

        self._publish_docking(
            {
                "active": True,
                "state": "SENDING",
                "request": request,
                "distanceRemaining": None,
                "lateralError": None,
                "yawError": None,
                "retryCount": 0,
                "result": None,
            }
        )
        try:
            future = client.send_goal_async(
                goal,
                feedback_callback=self._docking_feedback,
            )
            future.add_done_callback(self._docking_goal_response)
        except Exception as error:
            self._fail_docking(f"Could not send docking goal: {error}")
            return False, str(error)
        return True, f"Docking goal sent to {request['dock_id']}"

    def cancel_docking(self) -> tuple[bool, str]:
        with self._lock:
            pending = self._dock_goal_pending
            goal_handle = self._dock_goal_handle
            if pending:
                self._dock_cancel_requested = True

        if pending and goal_handle is None:
            self._publish_docking({"state": "CANCELING"})
            return True, "Docking cancellation queued"
        if goal_handle is None:
            return False, "There is no active docking goal"

        self._publish_docking({"state": "CANCELING"})
        try:
            goal_handle.cancel_goal_async()
        except Exception as error:
            self._fail_docking(f"Could not cancel docking: {error}")
            return False, str(error)
        return True, "Docking cancellation requested"

    def _docking_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self._fail_docking(f"Docking goal request failed: {error}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self._fail_docking("Docking goal was rejected", state="REJECTED")
            return

        with self._lock:
            self._dock_goal_pending = False
            self._dock_goal_handle = goal_handle
            cancel_requested = self._dock_cancel_requested

        self._publish_docking({"active": True, "state": "ACCEPTED"})
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._docking_result)
        if cancel_requested:
            goal_handle.cancel_goal_async()
            self._publish_docking({"state": "CANCELING"})

    def _docking_feedback(self, message) -> None:
        feedback = message.feedback
        state = self._dock_feedback_states.get(
            feedback.state,
            f"STATE_{feedback.state}",
        )
        self._publish_docking(
            {
                "active": True,
                "state": state,
                "distanceRemaining": round(float(feedback.distance_remaining), 4),
                "lateralError": round(float(feedback.lateral_error), 4),
                "yawError": round(float(feedback.yaw_error), 4),
                "retryCount": int(feedback.retry_count),
            }
        )

    def _docking_result(self, future) -> None:
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            goal_status = self._dock_goal_statuses.get(
                wrapped_result.status,
                f"STATUS_{wrapped_result.status}",
            )
        except Exception as error:
            self._fail_docking(f"Docking result failed: {error}")
            return

        with self._lock:
            self._dock_goal_handle = None
            self._dock_goal_pending = False
            self._dock_cancel_requested = False

        success = bool(result.success)
        self._publish_docking(
            {
                "active": False,
                "state": "SUCCEEDED" if success else goal_status,
                "result": {
                    "success": success,
                    "errorCode": int(result.error_code),
                    "message": str(result.message),
                    "goalStatus": goal_status,
                },
            }
        )

    def _fail_docking(self, message: str, state: str = "ERROR") -> None:
        with self._lock:
            self._dock_goal_handle = None
            self._dock_goal_pending = False
            self._dock_cancel_requested = False
        self._publish_docking(
            {
                "active": False,
                "state": state,
                "result": {
                    "success": False,
                    "errorCode": -1,
                    "message": message,
                    "goalStatus": state,
                },
            }
        )

    def _publish_docking(self, patch: dict) -> None:
        patch["updatedAt"] = utc_now()
        snapshot = robot_state.update({"docking": patch})
        self._publish(snapshot)


def create_monitor(
    loop: asyncio.AbstractEventLoop,
    dock_action_name: str,
) -> RosMonitor:
    snapshot_interval_seconds = 5.0
    last_snapshot_at = 0.0

    def publish(snapshot: dict) -> None:
        nonlocal last_snapshot_at

        asyncio.run_coroutine_threadsafe(ws_manager.broadcast(snapshot), loop)

        now = monotonic()
        if now - last_snapshot_at >= snapshot_interval_seconds:
            last_snapshot_at = now
            asyncio.run_coroutine_threadsafe(save_snapshot(snapshot), loop)

    return RosMonitor(publish, dock_action_name)
