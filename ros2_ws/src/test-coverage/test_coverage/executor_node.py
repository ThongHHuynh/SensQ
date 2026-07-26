#!/usr/bin/env python3

from __future__ import annotations

import copy
import json
import math
import time
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def pose_distance(first, second) -> float:
    return math.hypot(
        second.pose.position.x - first.pose.position.x,
        second.pose.position.y - first.pose.position.y,
    )


def bounded_closest_pose_index(
    poses,
    start_index: int,
    end_index: int,
    robot_x: float,
    robot_y: float,
    search_distance_m: float,
) -> int:
    """Find the nearest pose without jumping ahead across adjacent lanes."""
    closest_index = start_index
    closest_distance = math.inf
    traversed = 0.0
    for index in range(start_index, end_index + 1):
        if index > start_index:
            traversed += pose_distance(poses[index - 1], poses[index])
            if traversed > search_distance_m:
                break
        distance = math.hypot(
            poses[index].pose.position.x - robot_x,
            poses[index].pose.position.y - robot_y,
        )
        if distance < closest_distance:
            closest_distance = distance
            closest_index = index
    return closest_index


def rewind_pose_index(
    poses,
    index: int,
    lower_bound: int,
    overlap_m: float,
) -> int:
    """Retain a short path overlap before a confirmed progress cursor."""
    rewind_index = index
    traversed = 0.0
    while rewind_index > lower_bound:
        edge = pose_distance(poses[rewind_index - 1], poses[rewind_index])
        if traversed + edge > overlap_m:
            break
        traversed += edge
        rewind_index -= 1
    return rewind_index


class ExecutionState(str, Enum):
    IDLE = "IDLE"
    ENTERING = "ENTERING"
    FOLLOWING = "FOLLOWING"
    SUCCEEDED = "SUCCEEDED"
    CANCELING = "CANCELING"
    CANCELED = "CANCELED"
    ERROR = "ERROR"


class CoverageExecutor(Node):
    def __init__(self) -> None:
        super().__init__("coverage_executor")
        self._declare_parameters()
        self._state = ExecutionState.IDLE
        self._path = None
        self._pending_path = None
        self._segments_by_path_size = {}
        self._pending_segment_starts = [0]
        self._segment_index = 0
        self._segment_progress_index = 0
        self._follow_retry_count = 0
        self._goal_handle = None
        self._last_progress_time = time.monotonic()
        self._auto_started = False

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._path_subscription = self.create_subscription(
            Path,
            self.get_parameter("path_topic").value,
            self._path_callback,
            latched_qos,
        )
        self._segment_subscription = self.create_subscription(
            Int32MultiArray,
            self.get_parameter("segment_topic").value,
            self._segment_callback,
            latched_qos,
        )
        self._status_publisher = self.create_publisher(
            String, self.get_parameter("status_topic").value, latched_qos
        )
        self._start_service = self.create_service(Trigger, "~/start", self._start)
        self._cancel_service = self.create_service(Trigger, "~/cancel", self._cancel)
        self._status_service = self.create_service(Trigger, "~/status", self._status)

        self._navigate_client = ActionClient(
            self, NavigateToPose, self.get_parameter("navigate_action").value
        )
        self._follow_client = ActionClient(
            self, FollowPath, self.get_parameter("follow_action").value
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._watchdog_timer = self.create_timer(1.0, self._watchdog)
        self._auto_timer = self.create_timer(1.0, self._maybe_auto_start)
        self._publish_state("Waiting for coverage path")

    def _declare_parameters(self) -> None:
        declarations = {
            "path_topic": "/test_coverage/path",
            "segment_topic": "/test_coverage/segment_starts",
            "status_topic": "/test_coverage/execution_status",
            "map_frame": "map",
            "robot_base_frame": "base_footprint",
            "navigate_action": "/navigate_to_pose",
            "follow_action": "/follow_path",
            "controller_id": "FollowPath",
            "goal_checker_id": "general_goal_checker",
            "entry_tolerance_m": 0.15,
            "action_server_timeout_s": 10.0,
            "progress_timeout_s": 45.0,
            "max_follow_retries": 2,
            "resume_search_window_m": 1.0,
            "retry_overlap_m": 0.30,
            "auto_start": False,
        }
        for name, default in declarations.items():
            self.declare_parameter(name, default)

    def _path_callback(self, path: Path) -> None:
        if len(path.poses) < 2:
            self.get_logger().error("Rejected coverage path with fewer than two poses")
            return
        self._path = copy.deepcopy(path)
        self.get_logger().info(f"Received coverage path with {len(path.poses)} poses")

    def _segment_callback(self, message: Int32MultiArray) -> None:
        if len(message.data) < 2:
            self.get_logger().warn("Ignored empty coverage segment metadata")
            return
        path_size = int(message.data[0])
        starts = sorted(set(int(value) for value in message.data[1:]))
        if (
            path_size < 2
            or not starts
            or starts[0] != 0
            or starts[-1] >= path_size
        ):
            self.get_logger().warn("Ignored invalid coverage segment metadata")
            return
        self._segments_by_path_size[path_size] = starts
        while len(self._segments_by_path_size) > 4:
            self._segments_by_path_size.pop(next(iter(self._segments_by_path_size)))
        self.get_logger().info(f"Received {len(starts)} FollowPath segments")

    def _start(self, _request, response):
        success, message = self._begin()
        response.success = success
        response.message = message
        return response

    def _maybe_auto_start(self) -> None:
        if (
            self._auto_started
            or not self.get_parameter("auto_start").value
            or self._path is None
        ):
            return
        success, message = self._begin()
        if success:
            self._auto_started = True
        else:
            self.get_logger().warn(message, throttle_duration_sec=5.0)

    def _begin(self) -> tuple[bool, str]:
        if self._path is None:
            return False, "No coverage path received"
        if self._state in {
            ExecutionState.ENTERING,
            ExecutionState.FOLLOWING,
            ExecutionState.CANCELING,
        }:
            return False, f"Executor is busy in {self._state.value}"

        timeout = float(self.get_parameter("action_server_timeout_s").value)
        if not self._navigate_client.wait_for_server(timeout_sec=timeout):
            return False, "NavigateToPose action server unavailable"
        if not self._follow_client.wait_for_server(timeout_sec=timeout):
            return False, "FollowPath action server unavailable"

        self._pending_path = copy.deepcopy(self._path)
        self._pending_segment_starts = self._segments_by_path_size.get(
            len(self._pending_path.poses),
            [0],
        )
        self._segment_index = 0
        self._segment_progress_index = self._pending_segment_starts[0]
        self._follow_retry_count = 0
        first_pose = self._pending_path.poses[0]
        if self._requires_entry(first_pose):
            self._send_entry(first_pose)
            return True, "NavigateToPose entry goal requested"
        self._send_follow_path(self._pending_path)
        return True, "Robot already at entry; FollowPath requested"

    def _requires_entry(self, first_pose) -> bool:
        try:
            transform = self._tf_buffer.lookup_transform(
                first_pose.header.frame_id
                or self.get_parameter("map_frame").value,
                self.get_parameter("robot_base_frame").value,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as error:
            self.get_logger().warn(f"Entry TF unavailable, using NavigateToPose: {error}")
            return True
        distance = math.hypot(
            transform.transform.translation.x - first_pose.pose.position.x,
            transform.transform.translation.y - first_pose.pose.position.y,
        )
        self.get_logger().info(f"Coverage entry is {distance:.2f} m away")
        return distance > float(self.get_parameter("entry_tolerance_m").value)

    def _send_entry(self, pose) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = copy.deepcopy(pose)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self._set_state(ExecutionState.ENTERING, "Navigating to coverage entry")
        future = self._navigate_client.send_goal_async(
            goal, feedback_callback=self._entry_feedback
        )
        future.add_done_callback(self._entry_goal_response)

    def _entry_goal_response(self, future) -> None:
        try:
            self._goal_handle = future.result()
        except Exception as error:
            self._set_state(
                ExecutionState.ERROR,
                f"NavigateToPose request failed: {error}",
            )
            return
        if not self._goal_handle.accepted:
            self._set_state(ExecutionState.ERROR, "NavigateToPose rejected entry goal")
            return
        result = self._goal_handle.get_result_async()
        result.add_done_callback(self._entry_result)

    def _entry_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as error:
            self._goal_handle = None
            self._set_state(
                ExecutionState.ERROR,
                f"NavigateToPose result failed: {error}",
            )
            return
        self._goal_handle = None
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            self._pending_path = None
            self._set_state(ExecutionState.CANCELED, "Coverage entry canceled")
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._pending_path = None
            self._set_state(
                ExecutionState.ERROR,
                f"NavigateToPose ended with status {wrapped.status}",
            )
            return
        if self._pending_path is None:
            self._set_state(ExecutionState.ERROR, "Entry succeeded without pending path")
            return
        self.get_logger().info("Coverage entry reached; dispatching FollowPath")
        self._send_follow_path(self._pending_path)

    def _send_follow_path(self, path: Path) -> None:
        self._pending_path = path
        self._send_current_segment()

    def _current_segment_bounds(self) -> tuple[int, int]:
        start_index = self._pending_segment_starts[self._segment_index]
        if self._segment_index + 1 < len(self._pending_segment_starts):
            end_index = self._pending_segment_starts[self._segment_index + 1]
        else:
            end_index = len(self._pending_path.poses) - 1
        return start_index, end_index

    def _send_current_segment(self, resume: bool = False) -> None:
        if self._pending_path is None:
            self._set_state(ExecutionState.ERROR, "No path available for FollowPath")
            return
        segment_start, end_index = self._current_segment_bounds()
        start_index = segment_start
        if resume:
            start_index = rewind_pose_index(
                self._pending_path.poses,
                self._segment_progress_index,
                segment_start,
                float(self.get_parameter("retry_overlap_m").value),
            )
            start_index = min(start_index, end_index - 1)
        if end_index <= start_index:
            self._set_state(ExecutionState.ERROR, "Invalid FollowPath segment")
            return

        goal = FollowPath.Goal()
        goal.path.header = copy.deepcopy(self._pending_path.header)
        goal.path.poses = copy.deepcopy(
            self._pending_path.poses[start_index:end_index + 1]
        )
        stamp = self.get_clock().now().to_msg()
        goal.path.header.stamp = stamp
        for pose in goal.path.poses:
            pose.header.frame_id = goal.path.header.frame_id
            pose.header.stamp = stamp
        goal.controller_id = str(self.get_parameter("controller_id").value)
        goal.goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self._set_state(
            ExecutionState.FOLLOWING,
            f"Following segment {self._segment_index + 1}/"
            f"{len(self._pending_segment_starts)} "
            f"(poses {start_index}-{end_index}"
            f"{', resumed' if resume else ''})",
        )
        future = self._follow_client.send_goal_async(
            goal, feedback_callback=self._follow_feedback
        )
        future.add_done_callback(self._follow_goal_response)

    def _follow_goal_response(self, future) -> None:
        try:
            self._goal_handle = future.result()
        except Exception as error:
            self._set_state(
                ExecutionState.ERROR,
                f"FollowPath request failed: {error}",
            )
            return
        if not self._goal_handle.accepted:
            self._set_state(ExecutionState.ERROR, "FollowPath rejected coverage path")
            return
        result = self._goal_handle.get_result_async()
        result.add_done_callback(self._follow_result)

    def _follow_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as error:
            self._goal_handle = None
            self._pending_path = None
            self._set_state(
                ExecutionState.ERROR,
                f"FollowPath result failed: {error}",
            )
            return
        self._goal_handle = None
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            if self._segment_index + 1 < len(self._pending_segment_starts):
                self._segment_index += 1
                self._segment_progress_index = self._pending_segment_starts[
                    self._segment_index
                ]
                self._follow_retry_count = 0
                self._send_current_segment()
            else:
                self._pending_path = None
                self._set_state(
                    ExecutionState.SUCCEEDED,
                    "Coverage path completed successfully",
                )
        elif wrapped.status == GoalStatus.STATUS_CANCELED:
            self._pending_path = None
            self._set_state(ExecutionState.CANCELED, "Coverage execution canceled")
        else:
            maximum_retries = int(
                self.get_parameter("max_follow_retries").value
            )
            if self._follow_retry_count < maximum_retries:
                self._follow_retry_count += 1
                self.get_logger().warn(
                    f"FollowPath status {wrapped.status}; retrying segment "
                    f"{self._segment_index + 1} "
                    f"({self._follow_retry_count}/{maximum_retries})"
                )
                if not self._update_progress_from_tf():
                    self._pending_path = None
                    self._set_state(
                        ExecutionState.ERROR,
                        "Cannot safely resume segment without current TF",
                    )
                    return
                self._send_current_segment(resume=True)
            else:
                self._pending_path = None
                self._set_state(
                    ExecutionState.ERROR,
                    f"FollowPath ended with status {wrapped.status}",
                )

    def _entry_feedback(self, feedback_message) -> None:
        self._last_progress_time = time.monotonic()
        feedback = feedback_message.feedback
        self._publish_state(
            f"Entry distance remaining {feedback.distance_remaining:.2f} m"
        )

    def _follow_feedback(self, feedback_message) -> None:
        self._last_progress_time = time.monotonic()
        self._update_progress_from_tf()
        feedback = feedback_message.feedback
        self._publish_state(
            f"Segment {self._segment_index + 1}/"
            f"{len(self._pending_segment_starts)}, "
            f"remaining {feedback.distance_to_goal:.2f} m, "
            f"speed {feedback.speed:.2f} m/s"
        )

    def _update_progress_from_tf(self) -> bool:
        if self._pending_path is None:
            return False
        frame_id = (
            self._pending_path.header.frame_id
            or self.get_parameter("map_frame").value
        )
        try:
            transform = self._tf_buffer.lookup_transform(
                frame_id,
                self.get_parameter("robot_base_frame").value,
                rclpy.time.Time(),
            )
        except TransformException as error:
            self.get_logger().warn(
                f"Cannot update coverage progress from TF: {error}",
                throttle_duration_sec=5.0,
            )
            return False

        segment_start, segment_end = self._current_segment_bounds()
        cursor = min(
            max(self._segment_progress_index, segment_start),
            segment_end,
        )
        candidate = bounded_closest_pose_index(
            self._pending_path.poses,
            cursor,
            segment_end,
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            float(self.get_parameter("resume_search_window_m").value),
        )
        self._segment_progress_index = max(cursor, candidate)
        return True

    def _cancel(self, _request, response):
        if self._goal_handle is None:
            response.success = False
            response.message = "No active Nav2 goal"
            return response
        self._set_state(ExecutionState.CANCELING, "Cancel requested")
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(lambda _: self.get_logger().info("Cancel sent"))
        response.success = True
        response.message = "Cancel requested"
        return response

    def _status(self, _request, response):
        response.success = True
        response.message = self._state.value
        return response

    def _watchdog(self) -> None:
        if self._state not in {ExecutionState.ENTERING, ExecutionState.FOLLOWING}:
            return
        timeout = float(self.get_parameter("progress_timeout_s").value)
        if timeout <= 0.0 or time.monotonic() - self._last_progress_time <= timeout:
            return
        self.get_logger().error("Coverage progress watchdog expired")
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._set_state(ExecutionState.ERROR, "Progress watchdog expired")

    def _set_state(self, state: ExecutionState, detail: str) -> None:
        self._state = state
        self._last_progress_time = time.monotonic()
        self.get_logger().info(f"{state.value}: {detail}")
        self._publish_state(detail)

    def _publish_state(self, detail: str) -> None:
        message = String()
        message.data = json.dumps(
            {"state": self._state.value, "detail": detail},
            separators=(",", ":"),
        )
        self._status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
