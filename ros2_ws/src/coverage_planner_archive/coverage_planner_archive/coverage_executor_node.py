#!/usr/bin/env python3

import copy
import math
import time
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path, OccupancyGrid
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.action import ActionClient
import rclpy.duration
import rclpy.time
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from coverage_planner_archive.coverage_tracker import CoverageTracker
from coverage_planner_archive.grid_utils import (
    map_to_pixel,
    meters_to_pixels,
    occupancy_grid_to_array,
)


class CoverageState(Enum):
    IDLE = auto()
    PLANNING = auto()
    EXECUTING = auto()
    PAUSING = auto()
    PAUSED = auto()
    CANCELING = auto()
    COMPLETED = auto()
    ERROR = auto()


class ExecutionPhase(Enum):
    NONE = auto()
    ENTERING_PATH = auto()
    FOLLOWING_PATH = auto()


def path_length(path):
    """Return the planar arc length of a nav_msgs/Path."""
    return sum(
        math.hypot(
            end.pose.position.x - start.pose.position.x,
            end.pose.position.y - start.pose.position.y,
        )
        for start, end in zip(path.poses, path.poses[1:])
    )


def requires_path_entry(robot_position, target_pose, tolerance_m):
    """Return whether NavigateToPose is needed before FollowPath."""
    if robot_position is None:
        return True
    tolerance_m = max(0.0, float(tolerance_m))
    return math.hypot(
        robot_position[0] - target_pose.pose.position.x,
        robot_position[1] - target_pose.pose.position.y,
    ) > tolerance_m


def trim_path(path, start_distance_m):
    """Return a path beginning at an interpolated arc-length position."""
    if not path.poses:
        return Path()

    trimmed = Path()
    trimmed.header = copy.deepcopy(path.header)
    start_distance_m = max(0.0, float(start_distance_m))

    if start_distance_m <= 0.0 or len(path.poses) == 1:
        trimmed.poses = copy.deepcopy(path.poses)
        return trimmed

    traversed = 0.0
    for index, (start, end) in enumerate(zip(path.poses, path.poses[1:])):
        dx = end.pose.position.x - start.pose.position.x
        dy = end.pose.position.y - start.pose.position.y
        segment_length = math.hypot(dx, dy)
        segment_end = traversed + segment_length

        if start_distance_m <= segment_end:
            fraction = (
                0.0
                if segment_length == 0.0
                else min(1.0, (start_distance_m - traversed) / segment_length)
            )
            first = copy.deepcopy(start)
            first.header = copy.deepcopy(trimmed.header)
            first.pose.position.x += fraction * dx
            first.pose.position.y += fraction * dy
            if segment_length > 0.0:
                yaw = math.atan2(dy, dx)
                first.pose.orientation.x = 0.0
                first.pose.orientation.y = 0.0
                first.pose.orientation.z = math.sin(0.5 * yaw)
                first.pose.orientation.w = math.cos(0.5 * yaw)
            trimmed.poses.append(first)
            trimmed.poses.extend(copy.deepcopy(path.poses[index + 1:]))
            for pose in trimmed.poses:
                pose.header = copy.deepcopy(trimmed.header)
            return trimmed

        traversed = segment_end

    trimmed.poses = [copy.deepcopy(path.poses[-1])]
    trimmed.poses[0].header = copy.deepcopy(trimmed.header)
    return trimmed


def densify_path(path, max_spacing_m):
    """Interpolate path edges so Nav2 receives locally trackable spacing."""
    max_spacing_m = float(max_spacing_m)
    if not math.isfinite(max_spacing_m) or max_spacing_m <= 0.0:
        raise ValueError("max_spacing_m must be finite and positive")
    if len(path.poses) < 2:
        return copy.deepcopy(path)

    dense = Path()
    dense.header = copy.deepcopy(path.header)
    dense.poses.append(copy.deepcopy(path.poses[0]))
    dense.poses[0].header = copy.deepcopy(dense.header)

    for start, end in zip(path.poses, path.poses[1:]):
        dx = end.pose.position.x - start.pose.position.x
        dy = end.pose.position.y - start.pose.position.y
        segment_length = math.hypot(dx, dy)
        if segment_length == 0.0:
            continue

        divisions = max(1, int(math.ceil(segment_length / max_spacing_m)))
        yaw = math.atan2(dy, dx)
        for step in range(1, divisions + 1):
            fraction = step / divisions
            pose = copy.deepcopy(end)
            pose.header = copy.deepcopy(dense.header)
            pose.pose.position.x = start.pose.position.x + fraction * dx
            pose.pose.position.y = start.pose.position.y + fraction * dy
            if step < divisions:
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = math.sin(0.5 * yaw)
                pose.pose.orientation.w = math.cos(0.5 * yaw)
            dense.poses.append(pose)

    return dense


class CoverageExecutorNode(LifecycleNode):
    def __init__(self):
        super().__init__("coverage_executor_node")

        self.declare_parameter("map_topic", "/coverage/safe_map")
        self.declare_parameter("path_topic", "/coverage/path")
        self.declare_parameter("coverage_width_m", 0.3)
        self.declare_parameter("execution_timeout_s", 0.0)
        self.declare_parameter("progress_timeout_s", 30.0)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("action_server_timeout_s", 5.0)
        self.declare_parameter("resume_overlap_m", 0.15)
        self.declare_parameter("progress_update_threshold_m", 0.02)
        self.declare_parameter("controller_path_spacing_m", 0.20)
        self.declare_parameter("entry_tolerance_m", 0.25)

        self.state = CoverageState.IDLE
        self.current_path = None
        self.active_path = None
        self.latest_map = None
        self.coverage_tracker = None
        self.goal_handle = None
        self.goal_request_pending = False
        self.pending_cancel = False
        self.cancel_intent = None
        self.goal_sent_time = None
        self.last_progress_time = None
        self.path_total_distance = 0.0
        self.path_progress_distance = 0.0
        self.active_goal_start_distance = 0.0
        self.active_goal_path_length = 0.0
        self.active_phase = ExecutionPhase.NONE
        self.pending_follow_path = None
        self.pending_follow_start_distance = 0.0

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        # Subscribers.
        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter("path_topic").value,
            self.path_callback,
            qos,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.map_callback,
            qos,
        )

        # Publishers.
        self.status_pub = self.create_publisher(String, "/coverage/status", 10)
        self.progress_pub = self.create_publisher(Float32, "/coverage/progress", 10)

        # Services for control.
        self.start_srv = self.create_service(Trigger, "~/start", self.handle_start)
        self.pause_srv = self.create_service(Trigger, "~/pause", self.handle_pause)
        self.resume_srv = self.create_service(Trigger, "~/resume", self.handle_resume)
        self.cancel_srv = self.create_service(Trigger, "~/cancel", self.handle_cancel)

        # Nav2 action client.
        self.follow_path_client = ActionClient(
            self, FollowPath, "follow_path"
        )
        self.navigate_to_pose_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_callback)
        self.coverage_timer = self.create_timer(0.5, self.publish_coverage_progress)

        self.get_logger().info("Configured. Call ~/start to begin coverage.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self.state in (
            CoverageState.EXECUTING,
            CoverageState.PAUSING,
            CoverageState.CANCELING,
        ):
            self.cancel_intent = "cancel"
            self.transition_to(CoverageState.CANCELING)
            self.cancel_current_goal("lifecycle deactivation")
        else:
            self.transition_to(CoverageState.IDLE)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        if (
            self.goal_handle is not None
            or self.goal_request_pending
            or self.pending_cancel
        ):
            self.get_logger().error(
                "Cannot clean up while a Nav2 goal is active or pending."
            )
            return TransitionCallbackReturn.FAILURE

        self.current_path = None
        self.active_path = None
        self.latest_map = None
        self.coverage_tracker = None
        self.goal_handle = None
        self.goal_request_pending = False
        self.pending_cancel = False
        self.cancel_intent = None
        self.goal_sent_time = None
        self.last_progress_time = None
        self.path_total_distance = 0.0
        self.path_progress_distance = 0.0
        self.active_goal_start_distance = 0.0
        self.active_goal_path_length = 0.0
        self.active_phase = ExecutionPhase.NONE
        self.pending_follow_path = None
        self.pending_follow_start_distance = 0.0
        if hasattr(self, "watchdog_timer"):
            self.destroy_timer(self.watchdog_timer)
        if hasattr(self, "coverage_timer"):
            self.destroy_timer(self.coverage_timer)
        self.destroy_subscription(self.path_sub)
        self.destroy_subscription(self.map_sub)
        self.destroy_publisher(self.status_pub)
        self.destroy_publisher(self.progress_pub)
        self.destroy_service(self.start_srv)
        self.destroy_service(self.pause_srv)
        self.destroy_service(self.resume_srv)
        self.destroy_service(self.cancel_srv)
        self.tf_listener.unregister()
        self.follow_path_client.destroy()
        self.navigate_to_pose_client.destroy()
        return TransitionCallbackReturn.SUCCESS

    def path_callback(self, msg):
        self.current_path = msg
        self.get_logger().info(f"Received path with {len(msg.poses)} poses.")

    def map_callback(self, msg):
        self.latest_map = msg
        if self.coverage_tracker is None or self.state != CoverageState.EXECUTING:
            self.reset_coverage_tracker()

    def reset_coverage_tracker(self):
        if self.latest_map is None:
            return

        grid = occupancy_grid_to_array(self.latest_map)
        radius_m = 0.5 * float(self.get_parameter("coverage_width_m").value)
        radius_px = meters_to_pixels(radius_m, self.latest_map.info.resolution)
        self.coverage_tracker = CoverageTracker(grid, radius_px)

    def transition_to(self, new_state):
        self.get_logger().info(f"State: {self.state.name} → {new_state.name}")
        self.state = new_state
        if new_state == CoverageState.EXECUTING:
            now = time.monotonic()
            self.goal_sent_time = now
            self.last_progress_time = now
        status = String()
        status.data = new_state.name
        self.status_pub.publish(status)

    def handle_start(self, request, response):
        if self.current_path is None:
            response.success = False
            response.message = "No path available."
            return response

        if len(self.current_path.poses) < 2:
            response.success = False
            response.message = "FollowPath requires at least two poses"
            return response

        if self.state not in (CoverageState.IDLE, CoverageState.COMPLETED):
            response.success = False
            response.message = f"Cannot start from state {self.state.name}."
            return response

        self.active_path = copy.deepcopy(self.current_path)
        self.path_total_distance = path_length(self.active_path)
        self.path_progress_distance = 0.0
        self.reset_coverage_tracker()

        sent = self.start_path_with_safe_entry(
            self.active_path,
            start_distance_m=0.0,
        )
        if not sent:
            response.success = False
            response.message = "Failed to request FollowPath execution"
            return response

        self.transition_to(CoverageState.EXECUTING)
        response.success = True
        response.message = "Coverage started."
        return response

    def handle_pause(self, request, response):
        if self.state != CoverageState.EXECUTING:
            response.success = False
            response.message = f"Cannot pause from state {self.state.name}."
            return response

        self.cancel_intent = "pause"
        self.transition_to(CoverageState.PAUSING)
        self.cancel_current_goal("pause")
        response.success = True
        response.message = "Coverage pause requested."
        return response

    def handle_resume(self, request, response):
        if self.state != CoverageState.PAUSED:
            response.success = False
            response.message = f"Cannot resume from state {self.state.name}."
            return response

        if self.active_path is None:
            response.success = False
            response.message = "No active coverage path."
            return response

        overlap = max(0.0, float(self.get_parameter("resume_overlap_m").value))
        resume_distance = max(0.0, self.path_progress_distance - overlap)
        remaining = trim_path(self.active_path, resume_distance)
        if len(remaining.poses) < 2:
            self.transition_to(CoverageState.COMPLETED)
            response.success = True
            response.message = "Already complete."
            return response

        if not self.start_path_with_safe_entry(
            remaining,
            start_distance_m=resume_distance,
        ):
            response.success = False
            response.message = "Failed to resume FollowPath execution."
            return response

        self.transition_to(CoverageState.EXECUTING)
        response.success = True
        response.message = "Coverage resumed."
        return response

    def handle_cancel(self, request, response):
        if self.state == CoverageState.PAUSED:
            self.active_path = None
            self.transition_to(CoverageState.IDLE)
            response.success = True
            response.message = "Coverage cancelled."
        elif self.state in (CoverageState.EXECUTING, CoverageState.PAUSING):
            self.cancel_intent = "cancel"
            self.transition_to(CoverageState.CANCELING)
            self.cancel_current_goal("cancel")
            response.success = True
            response.message = "Coverage cancellation requested."
        else:
            response.success = False
            response.message = f"Nothing to cancel in state {self.state.name}."
        return response

    def start_path_with_safe_entry(self, path, start_distance_m):
        """Navigate to a distant path start before dispatching FollowPath."""
        target = path.poses[0]
        robot = self.lookup_robot_position(target.header.frame_id)
        tolerance = max(0.0, float(self.get_parameter("entry_tolerance_m").value))

        if robot is not None:
            distance = math.hypot(
                robot[0] - target.pose.position.x,
                robot[1] - target.pose.position.y,
            )
            if not requires_path_entry(robot, target, tolerance):
                self.get_logger().info(
                    f"Robot is {distance:.2f} m from path start; sending FollowPath."
                )
                return self.send_path(path, start_distance_m)
            self.get_logger().info(
                f"Robot is {distance:.2f} m from path start; navigating to entry."
            )
        else:
            self.get_logger().warn(
                "Robot pose unavailable; navigating to path entry before FollowPath."
            )

        self.pending_follow_path = copy.deepcopy(path)
        self.pending_follow_start_distance = max(0.0, float(start_distance_m))
        return self.send_entry_goal(target)

    def lookup_robot_position(self, frame_id):
        """Return the robot position in frame_id, or None when TF is unavailable."""
        target_frame = frame_id or self.get_parameter("map_frame").value
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                self.get_parameter("robot_base_frame").value,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as error:
            self.get_logger().warn(
                f"Path-entry TF lookup failed: {error}",
                throttle_duration_sec=5.0,
            )
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def send_entry_goal(self, pose):
        """Send NavigateToPose for safe coverage-path entry."""
        goal = NavigateToPose.Goal()
        goal.pose = copy.deepcopy(pose)
        return self.dispatch_goal(
            self.navigate_to_pose_client,
            goal,
            ExecutionPhase.ENTERING_PATH,
            "NavigateToPose",
            self.entry_feedback_callback,
        )

    def send_path(self, path, start_distance_m):
        """Send a nav_msgs/Path to Nav2's FollowPath action."""
        if path is None or len(path.poses) < 2:
            self.get_logger().error(
                "FollowPath requires at least two path poses."
            )
            self.transition_to(CoverageState.ERROR)
            return False

        try:
            controller_path = densify_path(
                path,
                self.get_parameter("controller_path_spacing_m").value,
            )
        except ValueError as error:
            self.get_logger().error(f"Invalid controller path spacing: {error}")
            self.transition_to(CoverageState.ERROR)
            return False

        if len(controller_path.poses) < 2:
            self.get_logger().error("Controller path has fewer than two poses.")
            self.transition_to(CoverageState.ERROR)
            return False

        # FollowPath expects a path plus Nav2 plugin identifiers.
        goal = FollowPath.Goal()
        goal.path = controller_path
        goal.controller_id = str(
            self.get_parameter("controller_id").value
        )
        goal.goal_checker_id = str(
            self.get_parameter("goal_checker_id").value
        )

        self.active_goal_start_distance = max(0.0, float(start_distance_m))
        self.active_goal_path_length = path_length(path)
        sent = self.dispatch_goal(
            self.follow_path_client,
            goal,
            ExecutionPhase.FOLLOWING_PATH,
            "FollowPath",
            self.feedback_callback,
        )
        if sent:
            self.pending_follow_path = None
            self.get_logger().info(
                f"Requested FollowPath with {len(controller_path.poses)} controller "
                f"poses from {len(path.poses)} coverage poses."
            )
        return sent

    def dispatch_goal(self, client, goal, phase, name, feedback_callback):
        """Dispatch one Nav2 action goal with shared mission bookkeeping."""
        timeout = float(self.get_parameter("action_server_timeout_s").value)
        if not client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(f"{name} action server is not available.")
            self.transition_to(CoverageState.ERROR)
            return False

        self.goal_handle = None
        self.goal_request_pending = True
        self.pending_cancel = False
        self.active_phase = phase
        self.goal_sent_time = time.monotonic()
        self.last_progress_time = self.goal_sent_time

        future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        future.add_done_callback(
            lambda result: self.goal_response_callback(result, phase, name)
        )
        return True

    def goal_response_callback(self, future, phase, name):
        self.goal_request_pending = False
        try:
            self.goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"{name} goal request failed: {e}")
            self.transition_to(CoverageState.ERROR)
            return

        if not self.goal_handle.accepted:
            self.get_logger().error(f"Goal rejected by {name}.")
            self.goal_handle = None
            self.transition_to(CoverageState.ERROR)
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.result_callback(result, phase)
        )

        if self.pending_cancel or self.state in (
            CoverageState.IDLE,
            CoverageState.PAUSING,
            CoverageState.CANCELING,
        ):
            self.goal_handle.cancel_goal_async()
            self.pending_cancel = False

    def entry_feedback_callback(self, feedback_msg):
        """Keep the watchdog alive while Nav2 approaches the path entry."""
        self.last_progress_time = time.monotonic()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        candidate = (
            self.active_goal_start_distance
            + self.active_goal_path_length
            - max(0.0, float(feedback.distance_to_goal))
        )
        candidate = min(self.path_total_distance, max(0.0, candidate))
        threshold = float(
            self.get_parameter("progress_update_threshold_m").value
        )
        if candidate >= self.path_progress_distance + threshold:
            self.path_progress_distance = candidate
            self.last_progress_time = time.monotonic()

        self.get_logger().debug(
            f"FollowPath remaining={feedback.distance_to_goal:.2f} m, "
            f"speed={feedback.speed:.2f} m/s."
        )

    def publish_coverage_progress(self):
        if (
            self.state != CoverageState.EXECUTING
            or self.active_phase != ExecutionPhase.FOLLOWING_PATH
            or self.coverage_tracker is None
        ):
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter("map_frame").value,
                self.get_parameter("robot_base_frame").value,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"Coverage TF lookup failed: {e}",
                throttle_duration_sec=5.0,
            )
            return

        px, py = map_to_pixel(
            transform.transform.translation.x,
            transform.transform.translation.y,
            self.latest_map,
        )
        self.coverage_tracker.mark_covered(px, py)
        self.publish_progress(self.coverage_tracker.coverage_percentage)

    def publish_progress(self, percentage):
        progress = Float32()
        progress.data = float(percentage)
        self.progress_pub.publish(progress)

    def result_callback(self, future, phase):
        try:
            wrapped_result = future.result()
        except Exception as e:
            action_name = (
                "NavigateToPose"
                if phase == ExecutionPhase.ENTERING_PATH
                else "FollowPath"
            )
            self.get_logger().error(f"{action_name} result failed: {e}")
            self.goal_handle = None
            self.active_phase = ExecutionPhase.NONE
            self.transition_to(CoverageState.ERROR)
            return

        status = wrapped_result.status
        intent = self.cancel_intent
        self.goal_handle = None
        self.goal_request_pending = False
        self.pending_cancel = False
        self.cancel_intent = None

        if (
            status == GoalStatus.STATUS_SUCCEEDED
            and phase == ExecutionPhase.ENTERING_PATH
        ):
            if intent == "pause":
                self.active_phase = ExecutionPhase.NONE
                self.transition_to(CoverageState.PAUSED)
                self.get_logger().info("Coverage paused at the path entry.")
                return
            if intent == "cancel":
                self.active_phase = ExecutionPhase.NONE
                self.active_path = None
                self.pending_follow_path = None
                self.transition_to(CoverageState.IDLE)
                self.get_logger().info("Coverage cancelled at the path entry.")
                return

            pending_path = self.pending_follow_path
            start_distance = self.pending_follow_start_distance
            self.active_phase = ExecutionPhase.NONE
            if pending_path is None:
                self.get_logger().error(
                    "Path entry succeeded, but no FollowPath goal is pending."
                )
                self.transition_to(CoverageState.ERROR)
                return
            self.get_logger().info(
                "Reached coverage path entry; starting FollowPath."
            )
            if not self.send_path(pending_path, start_distance):
                self.transition_to(CoverageState.ERROR)
            return

        self.active_phase = ExecutionPhase.NONE
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.path_progress_distance = self.path_total_distance
            self.transition_to(CoverageState.COMPLETED)
            self.get_logger().info("Coverage path completed successfully.")
        elif status == GoalStatus.STATUS_CANCELED and intent == "pause":
            self.transition_to(CoverageState.PAUSED)
            self.get_logger().info("Coverage path paused.")
        elif status == GoalStatus.STATUS_CANCELED and intent == "cancel":
            self.active_path = None
            self.pending_follow_path = None
            self.transition_to(CoverageState.IDLE)
            self.get_logger().info("Coverage path cancelled.")
        elif status == GoalStatus.STATUS_CANCELED and self.state == CoverageState.ERROR:
            self.get_logger().info("FollowPath stopped after executor error.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.transition_to(CoverageState.IDLE)
            self.get_logger().warn("FollowPath was canceled without an active intent.")
        elif status == GoalStatus.STATUS_ABORTED:
            action_name = (
                "NavigateToPose"
                if phase == ExecutionPhase.ENTERING_PATH
                else "FollowPath"
            )
            self.pending_follow_path = None
            self.get_logger().error(f"Nav2 aborted the {action_name} goal.")
            self.transition_to(CoverageState.ERROR)
        else:
            self.get_logger().error(
                f"FollowPath ended with unexpected status {status}."
            )
            self.transition_to(CoverageState.ERROR)

    def cancel_current_goal(self, reason):
        if self.goal_handle is None:
            self.pending_cancel = True
            self.get_logger().info(f"Queued goal cancellation for {reason}.")
            return

        self.goal_handle.cancel_goal_async()
        self.pending_cancel = False

    def watchdog_callback(self):
        if self.state != CoverageState.EXECUTING:
            return

        now = time.monotonic()
        execution_timeout = float(self.get_parameter("execution_timeout_s").value)
        progress_timeout = float(self.get_parameter("progress_timeout_s").value)

        if execution_timeout > 0.0 and self.goal_sent_time is not None:
            if now - self.goal_sent_time > execution_timeout:
                self.get_logger().error("Coverage execution timeout exceeded.")
                self.cancel_current_goal("execution timeout")
                self.transition_to(CoverageState.ERROR)
                return

        if progress_timeout > 0.0 and self.last_progress_time is not None:
            if now - self.last_progress_time > progress_timeout:
                self.get_logger().error("Coverage progress watchdog timeout exceeded.")
                self.cancel_current_goal("progress timeout")
                self.transition_to(CoverageState.ERROR)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
