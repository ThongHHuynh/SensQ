import asyncio
import json
import math
from datetime import datetime, timezone
from threading import Event, Lock, Thread
from time import monotonic
from typing import Callable

from .config import (
    CMD_VEL_TOPIC,
    COVERAGE_CANCEL_SERVICE,
    COVERAGE_EXECUTION_STATUS_TOPIC,
    COVERAGE_GENERATE_SERVICE,
    COVERAGE_PATH_TOPIC,
    COVERAGE_PLANNING_STATUS_TOPIC,
    COVERAGE_START_SERVICE,
    DOCK_RELOAD_SERVICE,
    INITIAL_POSE_TOPIC,
    JOINT_STATES_TOPIC,
    MAP_LOAD_SERVICE,
    MAP_TOPIC,
    ODOM_TOPIC,
    USE_SIM_TIME,
)
from .database import save_snapshot
from .state import robot_state, utc_now
from .websocket_manager import ws_manager


def yaw_from_quaternion(z: float, w: float) -> float:
    return math.degrees(math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z))


def yaw_from_full_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def planar_tag_normal_yaw(rotation, normal_sign: float) -> float | None:
    normal_x = 2.0 * (rotation.x * rotation.z + rotation.w * rotation.y)
    normal_y = 2.0 * (rotation.y * rotation.z - rotation.w * rotation.x)
    normal_x *= normal_sign
    normal_y *= normal_sign
    if math.hypot(normal_x, normal_y) < 0.25:
        return None
    return math.atan2(normal_y, normal_x)


def path_to_payload(message, max_points: int = 700) -> dict:
    poses = message.poses
    stride = max(1, math.ceil(len(poses) / max_points))
    points = [
        [round(float(pose.pose.position.x), 3), round(float(pose.pose.position.y), 3)]
        for pose in poses[::stride]
    ]
    if poses and (not points or points[-1] != [round(float(poses[-1].pose.position.x), 3), round(float(poses[-1].pose.position.y), 3)]):
        points.append([
            round(float(poses[-1].pose.position.x), 3),
            round(float(poses[-1].pose.position.y), 3),
        ])
    return {
        "frame": message.header.frame_id or "map",
        "totalPoses": len(poses),
        "points": points,
    }


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
    def __init__(
        self,
        publish: Callable[[dict], None],
        dock_action_name: str,
        tag_frames: dict[int, str] | None = None,
        tag_normal_sign: float = -1.0,
    ) -> None:
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
        self._node = None
        self._initial_pose_publisher = None
        self._initial_pose_type = None
        self._navigate_client = None
        self._navigate_type = None
        self._navigate_goal_handle = None
        self._navigate_goal_pending = False
        self._coverage_generate_client = None
        self._coverage_start_client = None
        self._coverage_cancel_client = None
        self._dock_reload_client = None
        self._map_load_client = None
        self._map_load_type = None
        self._trigger_type = None
        self._tag_frames = dict(tag_frames or {})
        self._tag_normal_sign = 1.0 if tag_normal_sign >= 0 else -1.0
        self._tag_last_seen: dict[int, float] = {}
        self._latest_tags: dict[int, dict] = {}
        self._has_map_pose = False

    def start(self) -> None:
        try:
            import rclpy
            from action_msgs.msg import GoalStatus
            from apriltag_msgs.msg import AprilTagDetectionArray
            from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
            from my_robot_docking_msgs.action import Dock
            from nav2_msgs.action import NavigateToPose
            from nav2_msgs.srv import LoadMap
            from nav_msgs.msg import Path
            from rclpy.action import ActionClient
            from rclpy.executors import ExternalShutdownException
            from rclpy.parameter import Parameter
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String
            from std_srvs.srv import Trigger
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
                node = rclpy.create_node(
                    "sensq_backend_monitor",
                    parameter_overrides=[
                        Parameter("use_sim_time", value=USE_SIM_TIME),
                    ],
                )
                latched_qos = QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                )
                cmd_vel_publisher = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
                initial_pose_publisher = node.create_publisher(
                    PoseWithCovarianceStamped,
                    INITIAL_POSE_TOPIC,
                    10,
                )
                dock_client = ActionClient(
                    node,
                    Dock,
                    self._dock_action_name,
                )
                with self._lock:
                    self._cmd_vel_publisher = cmd_vel_publisher
                    self._twist_type = Twist
                    self._node = node
                    self._initial_pose_publisher = initial_pose_publisher
                    self._initial_pose_type = PoseWithCovarianceStamped
                    self._ros_available = True
                    self._status_message = f"Publishing {CMD_VEL_TOPIC}"
                    self._dock_client = dock_client
                    self._dock_type = Dock
                    self._navigate_client = ActionClient(
                        node,
                        NavigateToPose,
                        "/navigate_to_pose",
                    )
                    self._navigate_type = NavigateToPose
                    self._coverage_generate_client = node.create_client(
                        Trigger, COVERAGE_GENERATE_SERVICE
                    )
                    self._coverage_start_client = node.create_client(
                        Trigger, COVERAGE_START_SERVICE
                    )
                    self._coverage_cancel_client = node.create_client(
                        Trigger, COVERAGE_CANCEL_SERVICE
                    )
                    self._dock_reload_client = node.create_client(
                        Trigger, DOCK_RELOAD_SERVICE
                    )
                    self._map_load_client = node.create_client(
                        LoadMap, MAP_LOAD_SERVICE
                    )
                    self._map_load_type = LoadMap
                    self._trigger_type = Trigger
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
                self._has_map_pose = True
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
                patch = {
                    "connection": {"lastHeartbeat": now, "launchState": "running"},
                    "navigation": {
                        "localization": (
                            "Receiving map TF"
                            if self._has_map_pose
                            else "Receiving odometry"
                        )
                    },
                    "hardwareStatus": {
                        "are_motors_ready": True,
                        "debug_message": f"Receiving {ODOM_TOPIC}",
                        "updatedAt": now,
                    },
                }
                if not self._has_map_pose:
                    patch["pose"] = {
                            "frame": msg.header.frame_id or "odom",
                            "x": round(float(pose.position.x), 3),
                            "y": round(float(pose.position.y), 3),
                            "yaw": round(yaw_from_quaternion(float(pose.orientation.z), float(pose.orientation.w)), 1),
                    }
                snapshot = robot_state.update(patch)
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
                        "navigation": {"localization": "Map frame active"},
                    }
                )
                snapshot = robot_state.update_device("SLAM", "online", f"Receiving {MAP_TOPIC}", MAP_TOPIC)
                self._publish(snapshot)

            def tag_cb(message) -> None:
                now = monotonic()
                with self._lock:
                    for detection in message.detections:
                        self._tag_last_seen[int(detection.id)] = now

            def update_tags() -> None:
                if tf_buffer is None:
                    return
                now = monotonic()
                with self._lock:
                    detected = dict(self._tag_last_seen)
                    frames = dict(self._tag_frames)
                observations = []
                latest = {}
                for tag_id, seen_at in detected.items():
                    age = now - seen_at
                    if age > 1.0:
                        continue
                    frame = frames.get(tag_id, f"tag_{tag_id}")
                    try:
                        base_transform = tf_buffer.lookup_transform(
                            "base_footprint", frame, rclpy.time.Time()
                        )
                    except Exception:
                        continue
                    base_translation = base_transform.transform.translation
                    base_yaw = planar_tag_normal_yaw(
                        base_transform.transform.rotation,
                        self._tag_normal_sign,
                    )
                    observation = {
                        "tagId": tag_id,
                        "tagFrame": frame,
                        "visible": True,
                        "ageSeconds": round(age, 2),
                        "distance": round(
                            math.hypot(base_translation.x, base_translation.y), 3
                        ),
                        "forward": round(float(base_translation.x), 3),
                        "lateral": round(float(base_translation.y), 3),
                        "bearingDegrees": round(
                            math.degrees(math.atan2(base_translation.y, base_translation.x)),
                            1,
                        ),
                        "orientationDegrees": (
                            round(math.degrees(base_yaw), 1)
                            if base_yaw is not None
                            else None
                        ),
                        "mapX": None,
                        "mapY": None,
                        "mapYaw": None,
                    }
                    try:
                        map_transform = tf_buffer.lookup_transform(
                            "map", frame, rclpy.time.Time()
                        )
                        map_translation = map_transform.transform.translation
                        map_yaw = planar_tag_normal_yaw(
                            map_transform.transform.rotation,
                            self._tag_normal_sign,
                        )
                        observation.update(
                            {
                                "mapX": round(float(map_translation.x), 3),
                                "mapY": round(float(map_translation.y), 3),
                                "mapYaw": (
                                    round(float(map_yaw), 4)
                                    if map_yaw is not None
                                    else None
                                ),
                            }
                        )
                    except Exception:
                        pass
                    observations.append(observation)
                    latest[tag_id] = observation
                observations.sort(key=lambda item: item["tagId"])
                with self._lock:
                    self._latest_tags = latest
                self._publish(robot_state.update({"tagDetections": observations}))

            def coverage_path_cb(message) -> None:
                self._publish(
                    robot_state.update(
                        {
                            "coverage": {
                                "path": path_to_payload(message),
                                "state": "READY",
                                "detail": f"Generated {len(message.poses)} coverage poses",
                                "updatedAt": utc_now(),
                            }
                        }
                    )
                )

            def planning_status_cb(message) -> None:
                try:
                    payload = json.loads(message.data)
                except (TypeError, json.JSONDecodeError):
                    payload = {"state": "ERROR", "message": str(message.data)}
                self._publish(
                    robot_state.update(
                        {
                            "coverage": {
                                "state": payload.get("state", "UNKNOWN"),
                                "detail": payload.get("message", "Coverage path ready"),
                                "metrics": payload,
                                "updatedAt": utc_now(),
                            }
                        }
                    )
                )

            def execution_status_cb(message) -> None:
                try:
                    payload = json.loads(message.data)
                except (TypeError, json.JSONDecodeError):
                    payload = {"state": "ERROR", "detail": str(message.data)}
                self._publish(
                    robot_state.update(
                        {
                            "coverage": {
                                "state": payload.get("state", "UNKNOWN"),
                                "detail": payload.get("detail", "Coverage status updated"),
                                "updatedAt": utc_now(),
                            },
                            "navigation": {
                                "state": (
                                    "Cleaning"
                                    if payload.get("state") in {"ENTERING", "FOLLOWING"}
                                    else "Idle"
                                )
                            },
                        }
                    )
                )

            if Odometry is not None:
                node.create_subscription(Odometry, ODOM_TOPIC, odom_cb, 10)
            if OccupancyGrid is not None:
                node.create_subscription(
                    OccupancyGrid,
                    MAP_TOPIC,
                    map_cb,
                    latched_qos,
                )
            if JointState is not None:
                node.create_subscription(JointState, JOINT_STATES_TOPIC, joint_cb, 10)
            if LaserScan is not None:
                node.create_subscription(LaserScan, "/scan", scan_cb, 10)
            if Imu is not None:
                node.create_subscription(Imu, "/imu", imu_cb, 10)
            if tf_buffer is not None:
                node.create_timer(0.5, update_map_pose)
                node.create_timer(0.2, update_tags)
            node.create_subscription(AprilTagDetectionArray, "/detections", tag_cb, 10)
            node.create_subscription(
                Path,
                COVERAGE_PATH_TOPIC,
                coverage_path_cb,
                latched_qos,
            )
            node.create_subscription(
                String,
                COVERAGE_PLANNING_STATUS_TOPIC,
                planning_status_cb,
                latched_qos,
            )
            node.create_subscription(
                String,
                COVERAGE_EXECUTION_STATUS_TOPIC,
                execution_status_cb,
                latched_qos,
            )
            try:
                rclpy.spin(node)
            except (ExternalShutdownException, KeyboardInterrupt):
                pass

        self._thread = Thread(target=run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        try:
            import rclpy
        except ImportError:
            return
        if rclpy.ok():
            rclpy.shutdown()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)

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

    def set_tag_frames(self, tag_frames: dict[int, str]) -> None:
        with self._lock:
            self._tag_frames = dict(tag_frames)

    def get_tag_observation(self, tag_id: int) -> dict | None:
        with self._lock:
            observation = self._latest_tags.get(tag_id)
            return dict(observation) if observation is not None else None

    def set_initial_pose(self, x: float, y: float, yaw_degrees: float) -> tuple[bool, str]:
        with self._lock:
            publisher = self._initial_pose_publisher
            message_type = self._initial_pose_type
            node = self._node
        if not self._ros_available or publisher is None or message_type is None or node is None:
            return False, "ROS initial-pose publisher is not initialized"

        yaw = math.radians(float(yaw_degrees))
        message = message_type()
        message.header.frame_id = "map"
        message.header.stamp = node.get_clock().now().to_msg()
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.orientation.z = math.sin(yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(yaw * 0.5)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = math.radians(15.0) ** 2
        publisher.publish(message)
        self._publish(
            robot_state.update(
                {
                    "navigation": {
                        "initialPose": {
                            "frame": "map",
                            "x": round(float(x), 3),
                            "y": round(float(y), 3),
                            "yaw": round(float(yaw_degrees), 1),
                        },
                        "localization": "Initial pose sent; waiting for AMCL",
                    }
                }
            )
        )
        return True, f"Published initial pose on {INITIAL_POSE_TOPIC}"

    def start_navigation(self, x: float, y: float, yaw_degrees: float) -> tuple[bool, str]:
        with self._lock:
            client = self._navigate_client
            action_type = self._navigate_type
            node = self._node
            busy = self._navigate_goal_pending or self._navigate_goal_handle is not None
        if busy:
            return False, "A navigation goal is already active"
        if not self._ros_available or client is None or action_type is None or node is None:
            return False, "Nav2 client is not initialized"
        if not client.server_is_ready():
            return False, "NavigateToPose action server is unavailable"

        yaw = math.radians(float(yaw_degrees))
        goal = action_type.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        pose = {"frame": "map", "x": float(x), "y": float(y), "yaw": float(yaw_degrees)}
        with self._lock:
            self._navigate_goal_pending = True
        self._publish_navigation_goal({"active": True, "state": "SENDING", "pose": pose, "message": "Sending Nav2 goal"})
        future = client.send_goal_async(goal, feedback_callback=self._navigation_feedback)
        future.add_done_callback(self._navigation_goal_response)
        return True, "Navigation goal sent"

    def cancel_navigation(self) -> tuple[bool, str]:
        with self._lock:
            goal_handle = self._navigate_goal_handle
        if goal_handle is None:
            return False, "There is no active navigation goal"
        goal_handle.cancel_goal_async()
        self._publish_navigation_goal({"state": "CANCELING", "message": "Cancellation requested"})
        return True, "Navigation cancellation requested"

    def coverage_services_ready(self) -> bool:
        with self._lock:
            clients = (self._coverage_generate_client, self._coverage_start_client)
        return all(client is not None and client.service_is_ready() for client in clients)

    def generate_coverage(self) -> tuple[bool, str]:
        success, message = self._call_trigger(
            self._coverage_generate_client,
            "Coverage planner",
            timeout=30.0,
        )
        if success:
            self._publish(robot_state.update({"coverage": {"plannerReady": True, "state": "READY", "detail": message}}))
        return success, message

    def execute_coverage(self) -> tuple[bool, str]:
        return self._call_trigger(self._coverage_start_client, "Coverage executor", timeout=15.0)

    def cancel_coverage(self) -> tuple[bool, str]:
        return self._call_trigger(self._coverage_cancel_client, "Coverage cancellation", timeout=5.0)

    def reload_docks(self) -> tuple[bool, str]:
        return self._call_trigger(self._dock_reload_client, "Dock database reload", timeout=5.0)

    def load_map(self, yaml_path: str) -> tuple[bool, str]:
        with self._lock:
            client = self._map_load_client
            service_type = self._map_load_type
        if client is None or service_type is None:
            return False, "Map loading client is not initialized"
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"Map service {MAP_LOAD_SERVICE} is unavailable"
        request = service_type.Request()
        request.map_url = str(yaml_path)
        result, error = self._wait_for_service_result(client, request, 10.0)
        if error:
            return False, error
        if int(result.result) != int(service_type.Response.RESULT_SUCCESS):
            return False, f"Map server returned result code {int(result.result)}"
        return True, f"Loaded map {yaml_path}"

    def _call_trigger(self, client, label: str, timeout: float) -> tuple[bool, str]:
        with self._lock:
            service_type = self._trigger_type
        if client is None or service_type is None:
            return False, f"{label} service client is not initialized"
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"{label} service is unavailable"
        result, error = self._wait_for_service_result(
            client,
            service_type.Request(),
            timeout,
        )
        if error:
            return False, error
        return bool(result.success), str(result.message)

    @staticmethod
    def _wait_for_service_result(client, request, timeout: float):
        completed = Event()
        outcome = {}
        future = client.call_async(request)

        def done(done_future) -> None:
            try:
                outcome["result"] = done_future.result()
            except Exception as error:
                outcome["error"] = str(error)
            completed.set()

        future.add_done_callback(done)
        if not completed.wait(timeout):
            return None, "ROS service call timed out"
        if "error" in outcome:
            return None, outcome["error"]
        return outcome.get("result"), None

    def _navigation_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self._finish_navigation(False, "ERROR", str(error))
            return
        if goal_handle is None or not goal_handle.accepted:
            self._finish_navigation(False, "REJECTED", "Nav2 rejected the goal")
            return
        with self._lock:
            self._navigate_goal_pending = False
            self._navigate_goal_handle = goal_handle
        self._publish_navigation_goal({"active": True, "state": "NAVIGATING", "message": "Nav2 goal accepted"})
        goal_handle.get_result_async().add_done_callback(self._navigation_result)

    def _navigation_feedback(self, message) -> None:
        self._publish_navigation_goal(
            {
                "active": True,
                "state": "NAVIGATING",
                "distanceRemaining": round(float(message.feedback.distance_remaining), 3),
            }
        )

    def _navigation_result(self, future) -> None:
        try:
            wrapped = future.result()
            succeeded = int(wrapped.status) == 4
            state = "SUCCEEDED" if succeeded else ("CANCELED" if int(wrapped.status) == 5 else "FAILED")
            message = f"NavigateToPose finished with status {int(wrapped.status)}"
        except Exception as error:
            succeeded, state, message = False, "ERROR", str(error)
        self._finish_navigation(succeeded, state, message)

    def _finish_navigation(self, succeeded: bool, state: str, message: str) -> None:
        with self._lock:
            self._navigate_goal_pending = False
            self._navigate_goal_handle = None
        self._publish_navigation_goal(
            {"active": False, "state": state, "message": message, "distanceRemaining": 0.0 if succeeded else None}
        )

    def _publish_navigation_goal(self, patch: dict) -> None:
        self._publish(robot_state.update({"navigation": {"goal": patch}}))

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
    tag_frames: dict[int, str] | None = None,
    tag_normal_sign: float = -1.0,
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

    return RosMonitor(
        publish,
        dock_action_name,
        tag_frames=tag_frames,
        tag_normal_sign=tag_normal_sign,
    )
