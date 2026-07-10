#!/usr/bin/env python3

import rclpy
import time
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.action import ActionClient
import rclpy.duration
import rclpy.time
from nav_msgs.msg import Path, OccupancyGrid
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from enum import Enum, auto

from coverage_planner.coverage_tracker import CoverageTracker
from coverage_planner.grid_utils import (
    map_to_pixel,
    meters_to_pixels,
    occupancy_grid_to_array,
)


class CoverageState(Enum):
    IDLE = auto()
    PLANNING = auto()
    EXECUTING = auto()
    PAUSED = auto()
    COMPLETED = auto()
    ERROR = auto()


class CoverageExecutorNode(LifecycleNode):
    def __init__(self):
        super().__init__("coverage_executor_node")

        self.declare_parameter("map_topic", "/coverage/safe_map")
        self.declare_parameter("path_topic", "/coverage/path")
        self.declare_parameter("coverage_width_m", 0.3)
        self.declare_parameter("max_retries_per_waypoint", 2)
        self.declare_parameter("execution_timeout_s", 0.0)
        self.declare_parameter("progress_timeout_s", 30.0)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("map_frame", "map")

        self.state = CoverageState.IDLE
        self.current_path = None
        self.latest_map = None
        self.coverage_tracker = None
        self.executed_count = 0
        self.goal_handle = None
        self.pending_cancel = False
        self.goal_sent_time = None
        self.last_progress_time = None

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
        self.follow_waypoints_client = ActionClient(
            self, FollowWaypoints, "follow_waypoints"
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
        self.transition_to(CoverageState.IDLE)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.current_path = None
        self.latest_map = None
        self.coverage_tracker = None
        self.executed_count = 0
        self.goal_handle = None
        self.pending_cancel = False
        self.goal_sent_time = None
        self.last_progress_time = None
        if hasattr(self, "watchdog_timer"):
            self.destroy_timer(self.watchdog_timer)
        if hasattr(self, "coverage_timer"):
            self.destroy_timer(self.coverage_timer)
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
        if self.current_path is None or not self.current_path.poses:
            response.success = False
            response.message = "No path available."
            return response

        if self.state != CoverageState.IDLE:
            response.success = False
            response.message = f"Cannot start from state {self.state.name}."
            return response

        self.transition_to(CoverageState.EXECUTING)
        self.executed_count = 0
        self.reset_coverage_tracker()
        self.send_waypoints(self.current_path.poses)
        response.success = True
        response.message = "Coverage started."
        return response

    def handle_pause(self, request, response):
        if self.state != CoverageState.EXECUTING:
            response.success = False
            response.message = f"Cannot pause from state {self.state.name}."
            return response

        self.cancel_current_goal("pause")
        self.transition_to(CoverageState.PAUSED)
        response.success = True
        response.message = "Coverage paused."
        return response

    def handle_resume(self, request, response):
        if self.state != CoverageState.PAUSED:
            response.success = False
            response.message = f"Cannot resume from state {self.state.name}."
            return response

        remaining = self.current_path.poses[self.executed_count:]
        if not remaining:
            self.transition_to(CoverageState.COMPLETED)
            response.success = True
            response.message = "Already complete."
            return response

        self.transition_to(CoverageState.EXECUTING)
        self.send_waypoints(remaining)
        response.success = True
        response.message = "Coverage resumed."
        return response

    def handle_cancel(self, request, response):
        if self.state in (CoverageState.EXECUTING, CoverageState.PAUSED):
            self.cancel_current_goal("cancel")
            self.transition_to(CoverageState.IDLE)
            response.success = True
            response.message = "Coverage cancelled."
        else:
            response.success = False
            response.message = f"Nothing to cancel in state {self.state.name}."
        return response

    def send_waypoints(self, poses):
        """Send waypoints to Nav2 FollowWaypoints action for smooth traversal."""
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowWaypoints action server not available.")
            self.transition_to(CoverageState.ERROR)
            return

        goal = FollowWaypoints.Goal()
        goal.poses = [pose for pose in poses]
        self.goal_handle = None
        self.pending_cancel = False
        self.goal_sent_time = time.monotonic()
        self.last_progress_time = self.goal_sent_time

        future = self.follow_waypoints_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"FollowWaypoints goal request failed: {e}")
            self.transition_to(CoverageState.ERROR)
            return

        if not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected by FollowWaypoints.")
            self.transition_to(CoverageState.ERROR)
            return

        if self.pending_cancel or self.state in (CoverageState.IDLE, CoverageState.PAUSED):
            self.goal_handle.cancel_goal_async()
            self.pending_cancel = False
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        current_wp = feedback_msg.feedback.current_waypoint
        total = len(self.current_path.poses)
        self.executed_count = current_wp
        self.last_progress_time = time.monotonic()
        if self.coverage_tracker is None:
            self.publish_progress(100.0 * current_wp / max(1, total))

    def publish_coverage_progress(self):
        if self.state != CoverageState.EXECUTING or self.coverage_tracker is None:
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

    def result_callback(self, future):
        if self.state not in (CoverageState.EXECUTING, CoverageState.PAUSED):
            return

        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"FollowWaypoints result failed: {e}")
            self.transition_to(CoverageState.ERROR)
            return

        missed = result.missed_waypoints
        total = len(self.current_path.poses)

        if missed:
            self.get_logger().warn(f"Missed {len(missed)} waypoints: {list(missed)}")

        self.executed_count = total
        self.transition_to(CoverageState.COMPLETED)
        self.get_logger().info(
            f"Coverage complete. {total - len(missed)}/{total} waypoints reached."
        )

    def cancel_current_goal(self, reason):
        if self.goal_handle is None:
            self.pending_cancel = True
            self.get_logger().info(f"Queued goal cancellation for {reason}.")
            return

        self.goal_handle.cancel_goal_async()
        self.goal_handle = None
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
