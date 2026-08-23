import asyncio
import json
import math
from datetime import datetime, timezone
from threading import Event, Lock, Thread
from time import monotonic
from typing import Callable

from .config import (
    CAMERA_TOPIC,
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
    MISSION_ACTION_NAME,
    ODOM_TOPIC,
    UNDOCK_ACTION_NAME,
    USE_SIM_TIME,
)
from .database import create_mission_record, save_snapshot
from .state import robot_state, utc_now
from .websocket_manager import ws_manager

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


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
        save_mission_record: Callable[[dict], None] | None = None,
    ) -> None:
        self._publish = publish
        self._save_mission_record = save_mission_record
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
        self._undock_action_name = UNDOCK_ACTION_NAME
        self._undock_client = None
        self._undock_type = None
        self._undock_goal_handle = None
        self._undock_goal_pending = False
        self._undock_cancel_requested = False
        self._undock_feedback_states = {}
        self._mission_action_name = MISSION_ACTION_NAME
        self._mission_client = None
        self._mission_type = None
        self._mission_goal_handle = None
        self._mission_goal_pending = False
        self._mission_cancel_requested = False
        self._mission_feedback_states = {}
        self._mission_start_time = None
        self._mission_request = None
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
        self._camera_lock = Lock()
        self._camera_frame: bytes | None = None
        self._camera_last_frame_at = 0.0
        self._camera_last_state_publish = 0.0

    def start(self) -> None:
        try:
            import rclpy
            from action_msgs.msg import GoalStatus
            from apriltag_msgs.msg import AprilTagDetectionArray
            from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
            from my_robot_docking_msgs.action import Dock, Mission, Undock
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
                    self._undock_client = ActionClient(
                        node,
                        Undock,
                        self._undock_action_name,
                    )
                    self._undock_type = Undock
                    self._undock_feedback_states = {
                        Undock.Feedback.IDLE: "IDLE",
                        Undock.Feedback.REVERSING: "REVERSING",
                        Undock.Feedback.ROTATING: "ROTATING",
                        Undock.Feedback.CLEARING: "CLEARING",
                        Undock.Feedback.COMPLETE: "COMPLETE",
                    }
                    self._mission_client = ActionClient(
                        node,
                        Mission,
                        self._mission_action_name,
                    )
                    self._mission_type = Mission
                    self._mission_feedback_states = {
                        Mission.Feedback.UNDOCKING: "UNDOCKING",
                        Mission.Feedback.NAVIGATING: "NAVIGATING",
                        Mission.Feedback.COVERING: "COVERING",
                        Mission.Feedback.RETURNING: "RETURNING",
                        Mission.Feedback.DOCKING: "DOCKING",
                        Mission.Feedback.COMPLETE: "COMPLETE",
                        Mission.Feedback.PAUSED: "PAUSED",
                        Mission.Feedback.ERROR: "ERROR",
                    }
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
                from sensor_msgs.msg import Image, JointState, LaserScan, Imu
            except ImportError:
                Image = None
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

            def camera_cb(msg) -> None:
                if msg.encoding == "bgr8":
                    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                elif msg.encoding == "rgb8":
                    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                    frame = frame[:, :, ::-1]
                elif msg.encoding == "mono8":
                    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                else:
                    return
                ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ok:
                    return
                with self._camera_lock:
                    self._camera_frame = encoded.tobytes()
                    self._camera_last_frame_at = monotonic()

                now = monotonic()
                if now - self._camera_last_state_publish > 1.0:
                    self._camera_last_state_publish = now
                    snapshot = robot_state.update(
                        {
                            "camera": {
                                "available": True,
                                "topic": CAMERA_TOPIC,
                                "width": int(msg.width),
                                "height": int(msg.height),
                            }
                        }
                    )
                    robot_state.update_device("Camera", "online", f"Receiving {CAMERA_TOPIC}", CAMERA_TOPIC)
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
            if Image is not None and cv2 is not None and np is not None:
                node.create_subscription(Image, CAMERA_TOPIC, camera_cb, 1)
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

    def get_camera_frame(self) -> bytes | None:
        with self._camera_lock:
            if self._camera_frame is None:
                return None
            if monotonic() - self._camera_last_frame_at > 2.0:
                return None
            return self._camera_frame

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

    def send_undock_goal(self, dock_id: str) -> tuple[bool, str]:
        with self._lock:
            client = self._undock_client
            undock_type = self._undock_type
            active = self._undock_goal_pending or self._undock_goal_handle is not None

        if active:
            return False, "An undocking goal is already active"
        if not self._ros_available or client is None or undock_type is None:
            return False, "ROS undocking client is not initialized"
        if not client.server_is_ready():
            return False, f"Undock action {self._undock_action_name} is unavailable"

        goal = undock_type.Goal()
        goal.dock_id = dock_id

        with self._lock:
            self._undock_goal_pending = True
            self._undock_cancel_requested = False

        self._publish_undocking(
            {
                "active": True,
                "state": "SENDING",
                "distanceCleared": None,
                "result": None,
            }
        )
        try:
            future = client.send_goal_async(
                goal,
                feedback_callback=self._undocking_feedback,
            )
            future.add_done_callback(self._undocking_goal_response)
        except Exception as error:
            self._fail_undocking(f"Could not send undocking goal: {error}")
            return False, str(error)
        return True, f"Undocking goal sent for {dock_id}"

    def cancel_undock(self) -> tuple[bool, str]:
        with self._lock:
            pending = self._undock_goal_pending
            goal_handle = self._undock_goal_handle
            if pending:
                self._undock_cancel_requested = True

        if pending and goal_handle is None:
            self._publish_undocking({"state": "CANCELING"})
            return True, "Undocking cancellation queued"
        if goal_handle is None:
            return False, "There is no active undocking goal"

        self._publish_undocking({"state": "CANCELING"})
        try:
            goal_handle.cancel_goal_async()
        except Exception as error:
            self._fail_undocking(f"Could not cancel undocking: {error}")
            return False, str(error)
        return True, "Undocking cancellation requested"

    def _undocking_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self._fail_undocking(f"Undocking goal request failed: {error}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self._fail_undocking("Undocking goal was rejected", state="REJECTED")
            return

        with self._lock:
            self._undock_goal_pending = False
            self._undock_goal_handle = goal_handle
            cancel_requested = self._undock_cancel_requested

        self._publish_undocking({"active": True, "state": "ACCEPTED"})
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._undocking_result)
        if cancel_requested:
            goal_handle.cancel_goal_async()
            self._publish_undocking({"state": "CANCELING"})

    def _undocking_feedback(self, message) -> None:
        feedback = message.feedback
        state = self._undock_feedback_states.get(
            feedback.state,
            f"STATE_{feedback.state}",
        )
        self._publish_undocking(
            {
                "active": True,
                "state": state,
                "distanceCleared": round(float(feedback.distance_cleared), 4),
            }
        )

    def _undocking_result(self, future) -> None:
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            goal_status = self._dock_goal_statuses.get(
                wrapped_result.status,
                f"STATUS_{wrapped_result.status}",
            )
        except Exception as error:
            self._fail_undocking(f"Undocking result failed: {error}")
            return

        with self._lock:
            self._undock_goal_handle = None
            self._undock_goal_pending = False
            self._undock_cancel_requested = False

        success = bool(result.success)
        self._publish_undocking(
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

    def _fail_undocking(self, message: str, state: str = "ERROR") -> None:
        with self._lock:
            self._undock_goal_handle = None
            self._undock_goal_pending = False
            self._undock_cancel_requested = False
        self._publish_undocking(
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

    def _publish_undocking(self, patch: dict) -> None:
        patch["updatedAt"] = utc_now()
        snapshot = robot_state.update({"undocking": patch})
        self._publish(snapshot)

    def start_mission(
        self, mission_type: str, dock_id: str, auto_dock_on_complete: bool
    ) -> tuple[bool, str]:
        with self._lock:
            client = self._mission_client
            mission_type_cls = self._mission_type
            active = self._mission_goal_pending or self._mission_goal_handle is not None

        if active:
            return False, "A mission is already active"
        if not self._ros_available or client is None or mission_type_cls is None:
            return False, "ROS mission client is not initialized"
        if not client.server_is_ready():
            return False, f"Mission action {self._mission_action_name} is unavailable"

        goal = mission_type_cls.Goal()
        goal.mission_type = mission_type
        goal.dock_id = dock_id
        goal.auto_dock_on_complete = auto_dock_on_complete

        with self._lock:
            self._mission_goal_pending = True
            self._mission_cancel_requested = False
            self._mission_start_time = monotonic()
            self._mission_request = {
                "mission_type": mission_type,
                "dock_id": dock_id,
                "started_at": datetime.now(timezone.utc),
            }

        self._publish_mission(
            {
                "active": True,
                "phase": "SENDING",
                "missionType": mission_type,
                "dockId": dock_id,
                "currentZone": None,
                "progressPercent": 0.0,
                "detail": "Sending mission goal",
                "elapsedSeconds": 0,
                "result": None,
            }
        )
        try:
            future = client.send_goal_async(goal, feedback_callback=self._mission_feedback)
            future.add_done_callback(self._mission_goal_response)
        except Exception as error:
            self._fail_mission(f"Could not send mission goal: {error}")
            return False, str(error)
        return True, f"Mission goal sent for {dock_id}"

    def cancel_mission(self) -> tuple[bool, str]:
        with self._lock:
            pending = self._mission_goal_pending
            goal_handle = self._mission_goal_handle
            if pending:
                self._mission_cancel_requested = True

        if pending and goal_handle is None:
            self._publish_mission({"phase": "CANCELING"})
            return True, "Mission cancellation queued"
        if goal_handle is None:
            return False, "There is no active mission"

        self._publish_mission({"phase": "CANCELING"})
        try:
            goal_handle.cancel_goal_async()
        except Exception as error:
            self._fail_mission(f"Could not cancel mission: {error}")
            return False, str(error)
        return True, "Mission cancellation requested"

    def _mission_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self._fail_mission(f"Mission goal request failed: {error}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self._fail_mission("Mission goal was rejected", phase="REJECTED")
            return

        with self._lock:
            self._mission_goal_pending = False
            self._mission_goal_handle = goal_handle
            cancel_requested = self._mission_cancel_requested

        self._publish_mission({"active": True, "phase": "ACCEPTED"})
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._mission_result)
        if cancel_requested:
            goal_handle.cancel_goal_async()
            self._publish_mission({"phase": "CANCELING"})

    def _mission_feedback(self, message) -> None:
        feedback = message.feedback
        phase = self._mission_feedback_states.get(
            feedback.phase,
            f"PHASE_{feedback.phase}",
        )
        with self._lock:
            elapsed = (
                monotonic() - self._mission_start_time
                if self._mission_start_time is not None
                else 0.0
            )
        self._publish_mission(
            {
                "active": True,
                "phase": phase,
                "currentZone": feedback.current_zone or None,
                "progressPercent": round(float(feedback.progress_percent), 1),
                "detail": feedback.detail,
                "elapsedSeconds": round(elapsed, 1),
            }
        )

    def _mission_result(self, future) -> None:
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            goal_status = self._dock_goal_statuses.get(
                wrapped_result.status,
                f"STATUS_{wrapped_result.status}",
            )
        except Exception as error:
            self._fail_mission(f"Mission result failed: {error}")
            return

        with self._lock:
            self._mission_goal_handle = None
            self._mission_goal_pending = False
            self._mission_cancel_requested = False
            start_time = self._mission_start_time
            request = dict(self._mission_request or {})
            self._mission_start_time = None

        success = bool(result.success)
        elapsed = monotonic() - start_time if start_time is not None else 0.0
        self._publish_mission(
            {
                "active": False,
                "phase": "SUCCEEDED" if success else goal_status,
                "progressPercent": 100.0 if success else 0.0,
                "elapsedSeconds": round(elapsed, 1),
                "result": {
                    "success": success,
                    "areaCoveredM2": round(float(result.area_covered_m2), 2),
                    "durationSeconds": round(float(result.duration_seconds), 1),
                    "message": str(result.message),
                    "goalStatus": goal_status,
                },
            }
        )
        self._record_mission(request, success, result.area_covered_m2, result.duration_seconds, str(result.message))

    def _fail_mission(self, message: str, phase: str = "ERROR") -> None:
        with self._lock:
            self._mission_goal_handle = None
            self._mission_goal_pending = False
            self._mission_cancel_requested = False
            request = dict(self._mission_request or {})
            self._mission_start_time = None
        self._publish_mission(
            {
                "active": False,
                "phase": phase,
                "result": {
                    "success": False,
                    "areaCoveredM2": 0.0,
                    "durationSeconds": 0.0,
                    "message": message,
                    "goalStatus": phase,
                },
            }
        )
        self._record_mission(request, False, 0.0, 0.0, message)

    def _record_mission(
        self, request: dict, success: bool, area_covered_m2: float, duration_seconds: float, message: str
    ) -> None:
        if self._save_mission_record is None or not request:
            return
        self._save_mission_record(
            {
                "mission_type": request.get("mission_type", ""),
                "dock_id": request.get("dock_id", ""),
                "started_at": request.get("started_at", datetime.now(timezone.utc)),
                "success": success,
                "area_covered_m2": float(area_covered_m2),
                "duration_seconds": float(duration_seconds),
                "message": message,
            }
        )

    def _publish_mission(self, patch: dict) -> None:
        patch["updatedAt"] = utc_now()
        snapshot = robot_state.update({"mission": patch})
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

    def save_mission_record(record: dict) -> None:
        asyncio.run_coroutine_threadsafe(
            create_mission_record(
                mission_type=record["mission_type"],
                dock_id=record["dock_id"],
                started_at=record["started_at"],
                success=record["success"],
                area_covered_m2=record["area_covered_m2"],
                duration_seconds=record["duration_seconds"],
                message=record["message"],
            ),
            loop,
        )

    return RosMonitor(
        publish,
        dock_action_name,
        tag_frames=tag_frames,
        tag_normal_sign=tag_normal_sign,
        save_mission_record=save_mission_record,
    )
