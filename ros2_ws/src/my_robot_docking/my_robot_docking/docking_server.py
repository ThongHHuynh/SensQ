"""Custom Nav2 staging and AprilTag visual docking action server."""

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from my_robot_docking.dock_database import DockDatabase, DockDatabaseError
from my_robot_docking.docking_geometry import (
    Pose2D,
    relative_control_error,
    reverse_target_from_tag,
    scan_sector_clearances,
    target_pose_from_tag,
)
from my_robot_docking.docking_visualization import (
    DockingStage,
    DockingVisualizer,
)
from my_robot_docking.tag_tracker import TagTracker
from my_robot_docking_msgs.action import Dock


class NavigationOutcome:
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    CANCELLED = 'cancelled'


class ApproachOutcome:
    SUCCEEDED = 'succeeded'
    TAG_NOT_FOUND = 'tag_not_found'
    TAG_LOST = 'tag_lost'
    TIMED_OUT = 'timed_out'
    SAFETY_STOP = 'safety_stop'
    CONTROL_FAILED = 'control_failed'
    CANCELLED = 'cancelled'


class DockingServer(Node):
    """Orchestrate Nav2 staging and local AprilTag visual servoing."""

    def __init__(self):
        super().__init__('docking_server')
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        if not self.dock_database_file:
            raise RuntimeError('dock_database_file parameter is empty')

        self.database = DockDatabase(
            self.dock_database_file,
            self.legacy_predocking_offset,
        )
        self.callback_group = ReentrantCallbackGroup()

        self._goal_lock = threading.Lock()
        self._busy = False
        self._active_dock_goal = None
        self._active_nav_goal = None
        self._navigation_feedback_valid = False
        self._predocking_acceptance_reached = False
        self._last_navigation_feedback_time = 0.0
        self._navigation_feedback_state = Dock.Feedback.NAVIGATING
        self._last_odom_received_ns = None
        self._last_scan_received_ns = None
        self._odom_lock = threading.Lock()
        self._odom_pose = None
        self._odom_frame_id = 'odom'
        self._front_clearance = math.inf
        self._rear_clearance = math.inf
        self._rotation_clearance = math.inf
        self._last_command = Twist()
        self._last_command_time = time.monotonic()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tag_tracker = TagTracker(
            node=self,
            tf_buffer=self.tf_buffer,
            base_frame=self.base_frame,
            detections_topic=self.detections_topic,
            tag_timeout=self.tag_timeout,
            transform_timeout=self.transform_timeout,
            normal_sign=self.tag_normal_sign,
            callback_group=self.callback_group,
        )
        self.visualizer = DockingVisualizer(
            node=self,
            enabled=self.visualization_enabled,
            topic=self.visualization_topic,
            line_width=self.visualization_line_width,
            minimum_point_distance=(
                self.visualization_min_point_distance
            ),
            publish_rate=self.visualization_publish_rate,
            maximum_points=self.visualization_maximum_points,
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_to_pose_action,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            Dock,
            self.dock_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self.callback_group,
        )
        self.reload_service = self.create_service(
            Trigger,
            '~/reload_database',
            self._reload_database,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f'Docking server ready: action={self.dock_action}, '
            f'docks={self.database.ids()}'
        )

    def _reload_database(self, _request, response):
        with self._goal_lock:
            if self._busy:
                response.success = False
                response.message = 'Cannot reload docks while docking is active'
                return response
            try:
                database = DockDatabase(
                    self.dock_database_file,
                    self.legacy_predocking_offset,
                )
            except DockDatabaseError as error:
                response.success = False
                response.message = str(error)
                return response
            self.database = database

        response.success = True
        response.message = f'Reloaded docks: {", ".join(self.database.ids())}'
        self.get_logger().info(response.message)
        return response

    def _declare_parameters(self):
        declarations = {
            'dock_database_file': '',
            'dock_action': '/dock',
            'navigate_to_pose_action': '/navigate_to_pose',
            'detections_topic': '/detections',
            'odom_topic': '/odometry/filtered',
            'scan_topic': '/scan',
            'cmd_vel_topic': '/cmd_vel_dock',
            'base_frame': 'base_footprint',
            'control_rate': 20.0,
            'max_retries': 3,
            'return_to_staging_on_retry': False,
            'legacy_predocking_offset': 0.5,
            'transform_timeout': 0.05,
            'tag_timeout': 0.5,
            'tag_loss_grace_period': 0.75,
            'tag_search_timeout': 15.0,
            'navigation_timeout': 120.0,
            'predocking_acceptance_distance': 0.25,
            'navigation_feedback_rate': 5.0,
            'approach_timeout': 30.0,
            'verification_duration': 0.3,
            'search_angular_speed': 0.2,
            'search_angle_limit': 1.57,
            'retry_backup_distance': 0.2,
            'retry_backup_speed': 0.05,
            'retry_rotation_angle': 0.35,
            'retry_rotation_speed': 0.2,
            'retry_recovery_timeout': 10.0,
            'rotation_stop_distance': 0.3,
            'max_linear_speed': 0.1,
            'max_angular_speed': 0.4,
            'max_linear_acceleration': 0.2,
            'max_angular_acceleration': 0.8,
            'distance_kp': 0.4,
            'heading_kp': 1.2,
            'lateral_kp': 0.8,
            'yaw_kp': 0.6,
            'coarse_approach_distance': 0.7,
            'rotate_in_place_threshold': 0.35,
            'distance_tolerance': 0.03,
            'lateral_tolerance': 0.03,
            'yaw_tolerance': 0.08,
            'minimum_tag_distance': 0.25,
            'front_stop_distance': 0.18,
            'front_sector_half_angle': 0.35,
            'rear_stop_distance': 0.18,
            'rear_sector_half_angle': 0.35,
            'scan_forward_angle': math.pi,
            'max_reverse_distance': 1.0,
            'sensor_timeout': 0.5,
            'command_timeout': 0.25,
            'stop_on_tag_loss': True,
            'require_fresh_odometry': True,
            'require_fresh_scan': True,
            'tag_normal_sign': -1.0,
            'visualization_enabled': True,
            'visualization_topic': '/docking/markers',
            'visualization_line_width': 0.03,
            'visualization_min_point_distance': 0.01,
            'visualization_publish_rate': 10.0,
            'visualization_maximum_points': 3000,
        }
        for name, default in declarations.items():
            self.declare_parameter(name, default)

    def _load_parameters(self):
        for name in (
            'dock_database_file',
            'dock_action',
            'navigate_to_pose_action',
            'detections_topic',
            'odom_topic',
            'scan_topic',
            'cmd_vel_topic',
            'base_frame',
            'control_rate',
            'max_retries',
            'return_to_staging_on_retry',
            'legacy_predocking_offset',
            'transform_timeout',
            'tag_timeout',
            'tag_loss_grace_period',
            'tag_search_timeout',
            'navigation_timeout',
            'predocking_acceptance_distance',
            'navigation_feedback_rate',
            'approach_timeout',
            'verification_duration',
            'search_angular_speed',
            'search_angle_limit',
            'retry_backup_distance',
            'retry_backup_speed',
            'retry_rotation_angle',
            'retry_rotation_speed',
            'retry_recovery_timeout',
            'rotation_stop_distance',
            'max_linear_speed',
            'max_angular_speed',
            'max_linear_acceleration',
            'max_angular_acceleration',
            'distance_kp',
            'heading_kp',
            'lateral_kp',
            'yaw_kp',
            'coarse_approach_distance',
            'rotate_in_place_threshold',
            'distance_tolerance',
            'lateral_tolerance',
            'yaw_tolerance',
            'minimum_tag_distance',
            'front_stop_distance',
            'front_sector_half_angle',
            'rear_stop_distance',
            'rear_sector_half_angle',
            'scan_forward_angle',
            'max_reverse_distance',
            'sensor_timeout',
            'command_timeout',
            'stop_on_tag_loss',
            'require_fresh_odometry',
            'require_fresh_scan',
            'tag_normal_sign',
            'visualization_enabled',
            'visualization_topic',
            'visualization_line_width',
            'visualization_min_point_distance',
            'visualization_publish_rate',
            'visualization_maximum_points',
        ):
            setattr(self, name, self.get_parameter(name).value)

    def _validate_parameters(self):
        positive = (
            'control_rate',
            'legacy_predocking_offset',
            'transform_timeout',
            'tag_timeout',
            'tag_loss_grace_period',
            'tag_search_timeout',
            'navigation_timeout',
            'navigation_feedback_rate',
            'approach_timeout',
            'verification_duration',
            'search_angular_speed',
            'search_angle_limit',
            'retry_backup_distance',
            'retry_backup_speed',
            'retry_rotation_angle',
            'retry_rotation_speed',
            'retry_recovery_timeout',
            'rotation_stop_distance',
            'max_linear_speed',
            'max_angular_speed',
            'max_linear_acceleration',
            'max_angular_acceleration',
            'distance_kp',
            'heading_kp',
            'lateral_kp',
            'yaw_kp',
            'distance_tolerance',
            'lateral_tolerance',
            'yaw_tolerance',
            'minimum_tag_distance',
            'front_stop_distance',
            'front_sector_half_angle',
            'rear_stop_distance',
            'rear_sector_half_angle',
            'max_reverse_distance',
            'sensor_timeout',
            'command_timeout',
            'visualization_line_width',
            'visualization_min_point_distance',
            'visualization_publish_rate',
        )
        invalid = [name for name in positive if getattr(self, name) <= 0.0]
        if invalid:
            raise ValueError(
                f'Docking parameters must be positive: {", ".join(invalid)}'
            )
        if self.max_retries < 0:
            raise ValueError('max_retries cannot be negative')
        if self.predocking_acceptance_distance < 0:
            raise ValueError('predocking_acceptance_distance cannot be negative')
        if self.retry_rotation_angle > math.pi:
            raise ValueError('retry_rotation_angle cannot exceed pi')
        if self.retry_backup_speed > self.max_linear_speed:
            raise ValueError('retry_backup_speed cannot exceed max_linear_speed')
        if self.retry_rotation_speed > self.max_angular_speed:
            raise ValueError(
                'retry_rotation_speed cannot exceed max_angular_speed'
            )
        if abs(self.tag_normal_sign) < 1e-6:
            raise ValueError('tag_normal_sign must be non-zero')
        if not math.isfinite(self.scan_forward_angle):
            raise ValueError('scan_forward_angle must be finite')
        if self.visualization_maximum_points <= 0:
            raise ValueError('visualization_maximum_points must be positive')

    def _odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        pose = Pose2D(float(position.x), float(position.y), float(yaw))
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
            self.get_logger().warning(
                'Ignoring non-finite odometry pose',
                throttle_duration_sec=2.0,
            )
            return
        with self._odom_lock:
            self._odom_pose = pose
            self._odom_frame_id = message.header.frame_id or 'odom'
        self._last_odom_received_ns = self.get_clock().now().nanoseconds
        self._visualization_call(
            'append_pose',
            pose,
            self._odom_frame_id,
        )

    def _scan_callback(self, message):
        self._front_clearance, self._rear_clearance = scan_sector_clearances(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
            self.scan_forward_angle,
            self.front_sector_half_angle,
            self.rear_sector_half_angle,
        )
        self._rotation_clearance = min(
            (
                float(value)
                for value in message.ranges
                if math.isfinite(value)
                and message.range_min <= value <= message.range_max
            ),
            default=math.inf,
        )
        self._last_scan_received_ns = self.get_clock().now().nanoseconds

    def _latest_odom_pose(self):
        with self._odom_lock:
            return self._odom_pose

    def _latest_odom_frame_id(self):
        with self._odom_lock:
            return self._odom_frame_id

    def _visualization_call(self, method, *args):
        visualizer = getattr(self, 'visualizer', None)
        if visualizer is None:
            return
        try:
            getattr(visualizer, method)(*args)
        except Exception as error:
            self.get_logger().warning(
                f'Docking visualization {method} failed: {error}',
                throttle_duration_sec=2.0,
            )

    def _goal_callback(self, request):
        try:
            dock = self.database.get(request.dock_id)
        except DockDatabaseError as error:
            self.get_logger().warning(str(error))
            return GoalResponse.REJECT

        if dock.final_distance < self.minimum_tag_distance:
            self.get_logger().warning(
                f'Dock {dock.dock_id} final_distance must be at least '
                'minimum_tag_distance'
            )
            return GoalResponse.REJECT

        if request.use_offset_override:
            if (
                not math.isfinite(request.final_distance)
                or request.final_distance < self.minimum_tag_distance
                or request.final_distance >= dock.staging_distance
                or not math.isfinite(request.lateral_offset)
                or not math.isfinite(request.yaw_offset)
            ):
                self.get_logger().warning('Invalid docking offset override')
                return GoalResponse.REJECT

        with self._goal_lock:
            if self._busy:
                self.get_logger().warning('Docking server is already busy')
                return GoalResponse.REJECT
            self._busy = True

        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        if self._active_nav_goal is not None:
            self._active_nav_goal.cancel_goal_async()
        self._stop_robot()
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self._active_dock_goal = goal_handle
        result = Dock.Result()
        request = goal_handle.request

        try:
            dock = self.database.get(request.dock_id)
            final_distance = (
                float(request.final_distance)
                if request.use_offset_override
                else dock.final_distance
            )
            lateral_offset = (
                float(request.lateral_offset)
                if request.use_offset_override
                else dock.lateral_offset
            )
            yaw_offset = (
                float(request.yaw_offset)
                if request.use_offset_override
                else dock.yaw_offset
            )
            self._visualization_call(
                'begin_goal',
                dock,
                final_distance,
                lateral_offset,
                yaw_offset,
            )

            prepared_at_staging = not request.navigate_to_staging_pose
            if request.navigate_to_staging_pose:
                navigation = self._navigate_to_predocking(
                    goal_handle,
                    dock,
                    lateral_offset,
                    yaw_offset,
                )
                if navigation == NavigationOutcome.CANCELLED:
                    return self._cancel_result(goal_handle)
                if navigation != NavigationOutcome.SUCCEEDED:
                    return self._abort_result(
                        goal_handle,
                        Dock.Result.NAVIGATION_FAILED,
                        'Failed to reach docking predocking pose',
                    )

            for attempt in range(self.max_retries + 1):
                if goal_handle.is_cancel_requested:
                    return self._cancel_result(goal_handle)

                observation, outcome, message = self._search_for_tag(
                    goal_handle,
                    dock,
                    attempt,
                )
                if outcome == ApproachOutcome.CANCELLED:
                    return self._cancel_result(goal_handle)
                if outcome == ApproachOutcome.SAFETY_STOP:
                    return self._abort_result(
                        goal_handle,
                        Dock.Result.SAFETY_STOP,
                        message,
                    )
                if observation is not None:
                    if not prepared_at_staging:
                        outcome, message = self._prepare_at_staging(
                            goal_handle,
                            dock,
                            lateral_offset,
                            yaw_offset,
                            attempt,
                        )
                        if outcome == ApproachOutcome.SUCCEEDED:
                            prepared_at_staging = True
                            observation = self.tag_tracker.latest(
                                dock.tag_id,
                                dock.tag_frame,
                            )
                            if observation is None:
                                outcome = ApproachOutcome.TAG_LOST
                                message = f'Lost tag {dock.tag_id}'

                    if (
                        prepared_at_staging
                        and observation is not None
                        and outcome in (None, ApproachOutcome.SUCCEEDED)
                    ):
                        outcome, message = self._approach_tag(
                            goal_handle,
                            dock,
                            observation,
                            final_distance,
                            lateral_offset,
                            yaw_offset,
                            attempt,
                        )

                if outcome == ApproachOutcome.SUCCEEDED:
                    self._stop_robot()
                    result.success = True
                    result.error_code = Dock.Result.NONE
                    mode = (
                        'Reverse-docked'
                        if dock.reverse_docking
                        else 'Docked'
                    )
                    result.message = f'{mode} at {dock.dock_id}'
                    self._visualization_call(
                        'finish',
                        True,
                        result.message,
                    )
                    goal_handle.succeed()
                    return result
                if outcome == ApproachOutcome.CANCELLED:
                    return self._cancel_result(goal_handle)
                if outcome == ApproachOutcome.SAFETY_STOP:
                    return self._abort_result(
                        goal_handle,
                        Dock.Result.SAFETY_STOP,
                        message,
                    )

                if attempt >= self.max_retries:
                    error_code = (
                        Dock.Result.TAG_NOT_FOUND
                        if outcome == ApproachOutcome.TAG_NOT_FOUND
                        else (
                            Dock.Result.TAG_LOST
                            if outcome == ApproachOutcome.TAG_LOST
                            else Dock.Result.CONTROL_FAILED
                        )
                    )
                    return self._abort_result(goal_handle, error_code, message)

                self._publish_feedback(
                    goal_handle,
                    Dock.Feedback.RETRYING,
                    retries=attempt + 1,
                )
                self._stop_robot()

                tag_failure = outcome in (
                    ApproachOutcome.TAG_NOT_FOUND,
                    ApproachOutcome.TAG_LOST,
                )
                if tag_failure:
                    recovery, recovery_message = self._recover_for_retry(
                        goal_handle,
                        attempt + 1,
                    )
                    if recovery == ApproachOutcome.CANCELLED:
                        return self._cancel_result(goal_handle)
                    if recovery != ApproachOutcome.SUCCEEDED:
                        error_code = (
                            Dock.Result.SAFETY_STOP
                            if recovery == ApproachOutcome.SAFETY_STOP
                            else Dock.Result.CONTROL_FAILED
                        )
                        return self._abort_result(
                            goal_handle,
                            error_code,
                            recovery_message,
                        )
                elif (
                    request.navigate_to_staging_pose
                    and self.return_to_staging_on_retry
                ):
                    navigation = self._navigate_to_predocking(
                        goal_handle,
                        dock,
                        lateral_offset,
                        yaw_offset,
                    )
                    if navigation == NavigationOutcome.CANCELLED:
                        return self._cancel_result(goal_handle)
                    if navigation != NavigationOutcome.SUCCEEDED:
                        return self._abort_result(
                            goal_handle,
                            Dock.Result.NAVIGATION_FAILED,
                            'Failed to return to predocking pose for retry',
                        )
                    prepared_at_staging = False

            return self._abort_result(
                goal_handle,
                Dock.Result.CONTROL_FAILED,
                'Docking ended unexpectedly',
            )
        except Exception as error:  # Keep the action from leaving motion active.
            self.get_logger().exception(f'Docking failed: {error}')
            return self._abort_result(
                goal_handle,
                Dock.Result.CONTROL_FAILED,
                str(error),
            )
        finally:
            self._stop_robot()
            with self._goal_lock:
                self._busy = False
            self._active_dock_goal = None
            self._active_nav_goal = None

    def _navigate_to_predocking(
        self,
        dock_goal,
        dock,
        lateral_offset,
        yaw_offset,
    ):
        self._visualization_call(
            'set_stage',
            DockingStage.NAVIGATION,
        )
        self._navigation_feedback_state = Dock.Feedback.NAVIGATING_PREDOCK
        self._publish_feedback(dock_goal, self._navigation_feedback_state)
        self._stop_robot()
        time.sleep(self.command_timeout + 0.05)
        self._navigation_feedback_valid = False
        self._predocking_acceptance_reached = False
        self._last_navigation_feedback_time = 0.0

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action is unavailable')
            return NavigationOutcome.FAILED

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = dock.global_frame
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        predocking_pose = target_pose_from_tag(
            Pose2D(
                dock.reference_x,
                dock.reference_y,
                dock.approach_yaw,
            ),
            dock.predocking_distance,
            lateral_offset,
            yaw_offset,
        )
        nav_goal.pose.pose.position.x = predocking_pose.x
        nav_goal.pose.pose.position.y = predocking_pose.y
        nav_goal.pose.pose.orientation.z = math.sin(predocking_pose.yaw * 0.5)
        nav_goal.pose.pose.orientation.w = math.cos(predocking_pose.yaw * 0.5)

        send_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self._navigation_feedback,
        )
        if not self._wait_for_future(send_future, dock_goal, 5.0):
            return (
                NavigationOutcome.CANCELLED
                if dock_goal.is_cancel_requested
                else NavigationOutcome.FAILED
            )

        nav_handle = send_future.result()
        if nav_handle is None or not nav_handle.accepted:
            return NavigationOutcome.FAILED

        self._active_nav_goal = nav_handle
        result_future = nav_handle.get_result_async()
        if not self._wait_for_future(
            result_future,
            dock_goal,
            self.navigation_timeout,
            cancel_nav=True,
        ):
            return (
                NavigationOutcome.CANCELLED
                if dock_goal.is_cancel_requested
                else NavigationOutcome.FAILED
            )

        wrapped_result = result_future.result()
        self._active_nav_goal = None
        if self._predocking_acceptance_reached:
            self.get_logger().info(
                'Predocking acceptance distance reached; switching to visual alignment'
            )
            return NavigationOutcome.SUCCEEDED
        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationOutcome.SUCCEEDED
        if wrapped_result.status == GoalStatus.STATUS_CANCELED:
            return NavigationOutcome.CANCELLED
        return NavigationOutcome.FAILED

    def _wait_for_future(self, future, dock_goal, timeout, cancel_nav=False):
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if dock_goal.is_cancel_requested:
                if cancel_nav and self._active_nav_goal is not None:
                    self._active_nav_goal.cancel_goal_async()
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                if cancel_nav and self._active_nav_goal is not None:
                    self._active_nav_goal.cancel_goal_async()
                return False
            completed.wait(timeout=min(remaining, 0.05))
        return future.done()

    def _navigation_feedback(self, message):
        goal = self._active_dock_goal
        if goal is None or not goal.is_active:
            return

        distance = float(message.feedback.distance_remaining)
        acceptance_enabled = self.predocking_acceptance_distance > 0.0
        if acceptance_enabled and math.isfinite(distance):
            if distance > self.predocking_acceptance_distance:
                self._navigation_feedback_valid = True
            if (
                self._navigation_feedback_valid
                and distance <= self.predocking_acceptance_distance
                and not self._predocking_acceptance_reached
            ):
                self._predocking_acceptance_reached = True
                if self._active_nav_goal is not None:
                    self._active_nav_goal.cancel_goal_async()

        now = time.monotonic()
        if (
            now - self._last_navigation_feedback_time
            < 1.0 / self.navigation_feedback_rate
        ):
            return
        self._last_navigation_feedback_time = now
        self._publish_feedback(
            goal,
            self._navigation_feedback_state,
            distance=distance,
        )

    def _search_for_tag(self, goal_handle, dock, retries):
        self._visualization_call(
            'set_stage',
            DockingStage.PREDOCK_ALIGNMENT,
        )
        self._publish_feedback(
            goal_handle,
            Dock.Feedback.SEARCHING,
            retries=retries,
        )
        deadline = time.monotonic() + self.tag_search_timeout
        direction = 1.0
        search_limit = (
            math.pi if dock.reverse_docking else self.search_angle_limit
        )
        direction_deadline = time.monotonic() + (
            search_limit / max(self.search_angular_speed, 1e-3)
        )
        period = 1.0 / self.control_rate

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                return None, ApproachOutcome.CANCELLED, 'Docking cancelled'

            observation = self.tag_tracker.latest(dock.tag_id, dock.tag_frame)
            if observation is not None:
                self._stop_robot()
                return observation, None, ''

            safety = self._sensor_safety_reason(require_scan=False)
            if safety is not None:
                self.get_logger().warning(safety)
                self._stop_robot()
                return None, ApproachOutcome.SAFETY_STOP, safety

            if time.monotonic() >= direction_deadline:
                direction *= -1.0
                direction_deadline = time.monotonic() + (
                    2.0 * search_limit
                    / max(self.search_angular_speed, 1e-3)
                )

            command = Twist()
            command.angular.z = direction * self.search_angular_speed
            self._publish_dock_command(command)
            time.sleep(period)

        self._stop_robot()
        return (
            None,
            ApproachOutcome.TAG_NOT_FOUND,
            f'Tag {dock.tag_id} was not found',
        )

    def _recover_for_retry(self, goal_handle, retry_number):
        """Back up and rotate using odometry before another tag search."""

        self._visualization_call('set_stage', DockingStage.RETRY)

        period = 1.0 / self.control_rate
        deadline = time.monotonic() + self.retry_recovery_timeout
        start_pose = self._latest_odom_pose()
        if start_pose is None:
            return ApproachOutcome.SAFETY_STOP, 'Odometry pose is unavailable'

        self.get_logger().info(
            f'Docking retry {retry_number}: backing up '
            f'{self.retry_backup_distance:.3f} m'
        )

        try:
            current_pose = start_pose
            while rclpy.ok() and time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    return ApproachOutcome.CANCELLED, 'Docking cancelled'

                safety = self._retry_recovery_safety_reason(rotating=False)
                if safety is not None:
                    return ApproachOutcome.SAFETY_STOP, safety

                current_pose = self._latest_odom_pose()
                if current_pose is None:
                    return (
                        ApproachOutcome.SAFETY_STOP,
                        'Odometry pose is unavailable',
                    )

                travelled = math.hypot(
                    current_pose.x - start_pose.x,
                    current_pose.y - start_pose.y,
                )
                if travelled >= self.retry_backup_distance:
                    break

                self._publish_feedback(
                    goal_handle,
                    Dock.Feedback.RETRYING,
                    distance=(self.retry_backup_distance - travelled),
                    retries=retry_number,
                )
                command = Twist()
                command.linear.x = -self.retry_backup_speed
                self._publish_dock_command(command)
                time.sleep(period)
            else:
                return (
                    ApproachOutcome.TIMED_OUT,
                    'Retry backup timed out',
                )

            self._stop_robot()
            time.sleep(period)

            rotation_offset = self._retry_rotation_offset(retry_number)
            target_yaw = self._normalize_angle(
                current_pose.yaw + rotation_offset
            )
            self.get_logger().info(
                f'Docking retry {retry_number}: rotating '
                f'{rotation_offset:.3f} rad'
            )

            while rclpy.ok() and time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    return ApproachOutcome.CANCELLED, 'Docking cancelled'

                safety = self._retry_recovery_safety_reason(rotating=True)
                if safety is not None:
                    return ApproachOutcome.SAFETY_STOP, safety

                current_pose = self._latest_odom_pose()
                if current_pose is None:
                    return (
                        ApproachOutcome.SAFETY_STOP,
                        'Odometry pose is unavailable',
                    )

                yaw_error = self._normalize_angle(
                    target_yaw - current_pose.yaw
                )
                if abs(yaw_error) <= self.yaw_tolerance:
                    return ApproachOutcome.SUCCEEDED, 'Retry recovery complete'

                self._publish_feedback(
                    goal_handle,
                    Dock.Feedback.RETRYING,
                    yaw=yaw_error,
                    retries=retry_number,
                )
                command = Twist()
                command.angular.z = self._clamp(
                    self.heading_kp * yaw_error,
                    -self.retry_rotation_speed,
                    self.retry_rotation_speed,
                )
                self._publish_dock_command(command)
                time.sleep(period)

            return ApproachOutcome.TIMED_OUT, 'Retry rotation timed out'
        finally:
            self._stop_robot()

    def _retry_recovery_safety_reason(self, rotating):
        reason = self._sensor_safety_reason(require_scan=True)
        if reason is not None:
            return reason
        if rotating:
            if self._rotation_clearance < self.rotation_stop_distance:
                return (
                    f'Obstacle at {self._rotation_clearance:.3f} m is inside '
                    f'the {self.rotation_stop_distance:.3f} m rotation '
                    'stop distance'
                )
            return None
        if self._rear_clearance < self.rear_stop_distance:
            return (
                f'Rear obstacle at {self._rear_clearance:.3f} m is inside '
                f'the {self.rear_stop_distance:.3f} m stop distance'
            )
        return None

    def _retry_rotation_offset(self, retry_number):
        direction = 1.0 if retry_number % 2 == 1 else -1.0
        return direction * self.retry_rotation_angle

    def _prepare_at_staging(
        self,
        goal_handle,
        dock,
        lateral_offset,
        yaw_offset,
        retries,
    ):
        self._visualization_call(
            'set_stage',
            DockingStage.PREDOCK_ALIGNMENT,
        )
        outcome, message = self._approach_forward_target(
            goal_handle,
            dock,
            dock.predocking_distance,
            lateral_offset,
            yaw_offset,
            retries,
            Dock.Feedback.ALIGNING_PREDOCK,
            Dock.Feedback.ALIGNING_PREDOCK,
            'Predocking pose verified',
            'predocking pose',
        )
        if outcome != ApproachOutcome.SUCCEEDED:
            return outcome, message

        self.get_logger().info(
            f'Predocking alignment complete for {dock.dock_id}; '
            'advancing to staging'
        )
        self._visualization_call(
            'set_stage',
            DockingStage.STAGING_APPROACH,
        )
        return self._approach_forward_target(
            goal_handle,
            dock,
            dock.staging_distance,
            lateral_offset,
            yaw_offset,
            retries,
            Dock.Feedback.MOVING_TO_STAGING,
            Dock.Feedback.VERIFYING_STAGING,
            'Staging pose verified',
            'staging pose',
        )

    def _approach_tag(
        self,
        goal_handle,
        dock,
        initial_observation,
        final_distance,
        lateral_offset,
        yaw_offset,
        retries,
    ):
        if dock.reverse_docking:
            self._visualization_call(
                'set_stage',
                DockingStage.REVERSE_FINAL,
            )
            return self._approach_reverse(
                goal_handle,
                dock,
                initial_observation,
                final_distance,
                lateral_offset,
                yaw_offset,
                retries,
            )

        self._visualization_call(
            'set_stage',
            DockingStage.FINAL_APPROACH,
        )
        return self._approach_forward_target(
            goal_handle,
            dock,
            final_distance,
            lateral_offset,
            yaw_offset,
            retries,
            None,
            Dock.Feedback.VERIFYING,
            'Dock pose verified',
            'final docking pose',
        )

    def _approach_forward_target(
        self,
        goal_handle,
        dock,
        target_distance,
        lateral_offset,
        yaw_offset,
        retries,
        moving_state,
        verifying_state,
        success_message,
        target_name,
    ):
        """Visually servo to one pose on the tag approach centerline."""

        deadline = time.monotonic() + self.approach_timeout
        period = 1.0 / self.control_rate
        verification_started = None
        last_tag_seen = time.monotonic()

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return ApproachOutcome.CANCELLED, 'Docking cancelled'

            observation = self.tag_tracker.latest(dock.tag_id, dock.tag_frame)
            if observation is None:
                self._stop_robot()
                if time.monotonic() - last_tag_seen >= self.tag_loss_grace_period:
                    return ApproachOutcome.TAG_LOST, f'Lost tag {dock.tag_id}'
                time.sleep(period)
                continue
            last_tag_seen = time.monotonic()
            live_target = target_pose_from_tag(
                Pose2D(
                    observation.x,
                    observation.y,
                    observation.normal_yaw,
                ),
                target_distance,
                lateral_offset,
                yaw_offset,
            )
            self._visualization_call(
                'update_live_target',
                live_target,
                self.base_frame,
            )

            safety = self._safety_reason(
                math.hypot(observation.x, observation.y),
                reverse=False,
            )
            if safety is not None:
                self._stop_robot()
                return ApproachOutcome.SAFETY_STOP, safety

            (
                command,
                distance_error,
                lateral_error,
                yaw_error,
                reached,
                overshot,
            ) = self._compute_command(
                observation,
                target_distance,
                lateral_offset,
                yaw_offset,
            )

            if overshot:
                self._stop_robot()
                return (
                    ApproachOutcome.CONTROL_FAILED,
                    f'Robot passed the configured {target_name}',
                )

            if reached:
                self._stop_robot()
                if verification_started is None:
                    verification_started = time.monotonic()
                self._publish_feedback(
                    goal_handle,
                    verifying_state,
                    distance_error,
                    lateral_error,
                    yaw_error,
                    retries,
                )
                if (
                    time.monotonic() - verification_started
                    >= self.verification_duration
                ):
                    return ApproachOutcome.SUCCEEDED, success_message
            else:
                verification_started = None
                state = moving_state
                if state is None:
                    state = (
                        Dock.Feedback.ALIGNING
                        if distance_error <= self.coarse_approach_distance
                        else Dock.Feedback.APPROACHING
                    )
                self._publish_feedback(
                    goal_handle,
                    state,
                    distance_error,
                    lateral_error,
                    yaw_error,
                    retries,
                )
                self._publish_dock_command(command)

            time.sleep(period)

        self._stop_robot()
        return (
            ApproachOutcome.TIMED_OUT,
            f'Approach to {target_name} timed out',
        )

    def _approach_reverse(
        self,
        goal_handle,
        dock,
        initial_observation,
        final_distance,
        lateral_offset,
        yaw_offset,
        retries,
    ):
        """Rotate 180 degrees, then back into a tag-anchored odom goal."""

        robot_pose = self._latest_odom_pose()
        if robot_pose is None:
            return ApproachOutcome.SAFETY_STOP, 'Odometry pose is unavailable'

        target = reverse_target_from_tag(
            robot_pose,
            initial_observation.x,
            initial_observation.y,
            initial_observation.normal_yaw,
            final_distance,
            lateral_offset,
            yaw_offset,
        )
        self._visualization_call(
            'update_live_target',
            target.base_pose,
            self._latest_odom_frame_id(),
            robot_pose,
        )
        initial_error = relative_control_error(
            robot_pose,
            target.base_pose,
            reverse=True,
        )
        if initial_error.distance > self.max_reverse_distance:
            return (
                ApproachOutcome.CONTROL_FAILED,
                f'Reverse target is {initial_error.distance:.3f} m away, '
                f'above the {self.max_reverse_distance:.3f} m limit',
            )

        self.get_logger().info(
            f'Reverse docking {dock.dock_id}: captured tag target; '
            'rotating 180 degrees before backing in'
        )
        deadline = time.monotonic() + self.approach_timeout
        period = 1.0 / self.control_rate
        rotating = True
        verification_started = None
        reverse_distance_travelled = 0.0
        previous_reverse_pose = None

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return ApproachOutcome.CANCELLED, 'Docking cancelled'

            sensor_reason = self._sensor_safety_reason(require_scan=True)
            if sensor_reason is not None:
                self._stop_robot()
                return ApproachOutcome.SAFETY_STOP, sensor_reason

            robot_pose = self._latest_odom_pose()
            if robot_pose is None:
                self._stop_robot()
                return (
                    ApproachOutcome.SAFETY_STOP,
                    'Odometry pose is unavailable',
                )
            self._visualization_call(
                'update_live_target',
                target.base_pose,
                self._latest_odom_frame_id(),
                robot_pose,
            )

            tag_distance = math.hypot(
                target.tag_x - robot_pose.x,
                target.tag_y - robot_pose.y,
            )
            if tag_distance < self.minimum_tag_distance:
                self._stop_robot()
                return (
                    ApproachOutcome.SAFETY_STOP,
                    'Tag is inside the minimum safe distance',
                )

            error = relative_control_error(
                robot_pose,
                target.base_pose,
                reverse=True,
            )

            if rotating:
                if abs(error.yaw) <= self.yaw_tolerance:
                    rotating = False
                    previous_reverse_pose = robot_pose
                    self._stop_robot()
                    self.get_logger().info(
                        'Reverse heading reached; beginning rear-first approach'
                    )
                    time.sleep(period)
                    continue

                command = Twist()
                command.angular.z = self._clamp(
                    self.heading_kp * error.yaw,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )
                self._publish_feedback(
                    goal_handle,
                    Dock.Feedback.ALIGNING,
                    error.distance,
                    error.lateral,
                    error.yaw,
                    retries,
                )
                self._publish_dock_command(command)
                time.sleep(period)
                continue

            if previous_reverse_pose is not None:
                reverse_distance_travelled += math.hypot(
                    robot_pose.x - previous_reverse_pose.x,
                    robot_pose.y - previous_reverse_pose.y,
                )
            previous_reverse_pose = robot_pose
            if reverse_distance_travelled > self.max_reverse_distance:
                self._stop_robot()
                return (
                    ApproachOutcome.CONTROL_FAILED,
                    'Maximum reverse travel distance exceeded',
                )

            safety = self._safety_reason(tag_distance, reverse=True)
            if safety is not None:
                self._stop_robot()
                return ApproachOutcome.SAFETY_STOP, safety

            command, reached, overshot = self._compute_reverse_command(error)
            if overshot:
                self._stop_robot()
                return (
                    ApproachOutcome.CONTROL_FAILED,
                    'Robot passed the configured reverse docking pose',
                )

            if reached:
                self._stop_robot()
                if verification_started is None:
                    verification_started = time.monotonic()
                self._publish_feedback(
                    goal_handle,
                    Dock.Feedback.VERIFYING,
                    error.distance,
                    error.lateral,
                    error.yaw,
                    retries,
                )
                if (
                    time.monotonic() - verification_started
                    >= self.verification_duration
                ):
                    return (
                        ApproachOutcome.SUCCEEDED,
                        'Reverse dock pose verified',
                    )
            else:
                verification_started = None
                state = (
                    Dock.Feedback.ALIGNING
                    if error.distance <= self.coarse_approach_distance
                    else Dock.Feedback.APPROACHING
                )
                self._publish_feedback(
                    goal_handle,
                    state,
                    error.distance,
                    error.lateral,
                    error.yaw,
                    retries,
                )
                self._publish_dock_command(command)

            time.sleep(period)

        self._stop_robot()
        return ApproachOutcome.TIMED_OUT, 'Reverse dock approach timed out'

    def _compute_reverse_command(self, error):
        reached = (
            error.distance <= self.distance_tolerance
            and abs(error.lateral) <= self.lateral_tolerance
            and abs(error.yaw) <= self.yaw_tolerance
        )
        overshot = (
            error.longitudinal
            < -max(self.distance_tolerance, 0.03)
            and error.distance > self.distance_tolerance
        )

        command = Twist()
        if reached or overshot:
            return command, reached, overshot

        if (
            abs(error.heading) <= self.rotate_in_place_threshold
            and abs(error.yaw) <= self.rotate_in_place_threshold
        ):
            linear_limit = self.max_linear_speed
            if error.distance <= self.coarse_approach_distance:
                linear_limit *= 0.5
            speed = self._clamp(
                self.distance_kp * max(0.0, error.longitudinal),
                0.0,
                linear_limit,
            )
            command.linear.x = -speed

        angular = (
            self.heading_kp * error.heading
            - self.lateral_kp * error.lateral
            + self.yaw_kp * error.yaw
        )
        command.angular.z = self._clamp(
            angular,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        return command, reached, overshot

    def _compute_command(
        self,
        observation,
        final_distance,
        lateral_offset,
        yaw_offset,
    ):
        desired_yaw = self._normalize_angle(
            observation.normal_yaw + yaw_offset
        )
        direction_x = math.cos(desired_yaw)
        direction_y = math.sin(desired_yaw)
        left_x = -direction_y
        left_y = direction_x

        target_x = (
            observation.x
            - final_distance * direction_x
            + lateral_offset * left_x
        )
        target_y = (
            observation.y
            - final_distance * direction_y
            + lateral_offset * left_y
        )
        distance_error = math.hypot(target_x, target_y)
        lateral_error = target_y
        yaw_error = desired_yaw
        heading_error = math.atan2(target_y, target_x)
        longitudinal_error = (
            target_x * direction_x + target_y * direction_y
        )

        reached = (
            distance_error <= self.distance_tolerance
            and abs(lateral_error) <= self.lateral_tolerance
            and abs(yaw_error) <= self.yaw_tolerance
        )
        overshot = (
            longitudinal_error < -max(self.distance_tolerance, 0.03)
            and distance_error > self.distance_tolerance
        )

        command = Twist()
        if reached or overshot:
            return (
                command,
                distance_error,
                lateral_error,
                yaw_error,
                reached,
                overshot,
            )

        if abs(heading_error) <= self.rotate_in_place_threshold:
            linear_limit = self.max_linear_speed
            if distance_error <= self.coarse_approach_distance:
                linear_limit *= 0.5
            command.linear.x = self._clamp(
                self.distance_kp * max(0.0, target_x),
                0.0,
                linear_limit,
            )

        angular = (
            self.heading_kp * heading_error
            + self.lateral_kp * lateral_error
            + self.yaw_kp * yaw_error
        )
        command.angular.z = self._clamp(
            angular,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        return (
            command,
            distance_error,
            lateral_error,
            yaw_error,
            reached,
            overshot,
        )

    def _sensor_safety_reason(self, require_scan=True):
        now = self.get_clock().now().nanoseconds
        timeout_ns = int(self.sensor_timeout * 1e9)
        if self.require_fresh_odometry and (
            self._last_odom_received_ns is None
            or now - self._last_odom_received_ns > timeout_ns
        ):
            return 'Odometry is missing or stale'
        if require_scan and self.require_fresh_scan and (
            self._last_scan_received_ns is None
            or now - self._last_scan_received_ns > timeout_ns
        ):
            return 'Laser scan is missing or stale'
        return None

    def _safety_reason(self, tag_distance, reverse):
        reason = self._sensor_safety_reason(require_scan=True)
        if reason is not None:
            return reason
        if tag_distance < self.minimum_tag_distance:
            return 'Tag is inside the minimum safe distance'

        clearance = self._rear_clearance if reverse else self._front_clearance
        stop_distance = (
            self.rear_stop_distance if reverse else self.front_stop_distance
        )
        direction = 'Rear' if reverse else 'Front'
        if clearance < stop_distance:
            return (
                f'{direction} obstacle at {clearance:.3f} m is inside '
                f'the {stop_distance:.3f} m stop distance'
            )
        return None

    def _publish_dock_command(self, command):
        now = time.monotonic()
        dt = max(now - self._last_command_time, 1.0 / self.control_rate)
        limited = Twist()
        limited.linear.x = self._rate_limit(
            command.linear.x,
            self._last_command.linear.x,
            self.max_linear_acceleration * dt,
        )
        limited.angular.z = self._rate_limit(
            command.angular.z,
            self._last_command.angular.z,
            self.max_angular_acceleration * dt,
        )
        limited.linear.x = self._clamp(
            limited.linear.x,
            -self.max_linear_speed,
            self.max_linear_speed,
        )
        limited.angular.z = self._clamp(
            limited.angular.z,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        self.cmd_vel_publisher.publish(limited)
        self._last_command = limited
        self._last_command_time = now

    def _stop_robot(self):
        stop = Twist()
        self.cmd_vel_publisher.publish(stop)
        self._last_command = stop
        self._last_command_time = time.monotonic()

    def _publish_feedback(
        self,
        goal_handle,
        state,
        distance=0.0,
        lateral=0.0,
        yaw=0.0,
        retries=0,
    ):
        feedback = Dock.Feedback()
        feedback.state = int(state)
        feedback.distance_remaining = float(distance)
        feedback.lateral_error = float(lateral)
        feedback.yaw_error = float(yaw)
        feedback.retry_count = int(retries)
        goal_handle.publish_feedback(feedback)

    def _abort_result(self, goal_handle, error_code, message):
        self._stop_robot()
        self._visualization_call('finish', False, message)
        result = Dock.Result()
        result.success = False
        result.error_code = int(error_code)
        result.message = message
        if goal_handle.is_active:
            goal_handle.abort()
        return result

    def _cancel_result(self, goal_handle):
        self._stop_robot()
        self._visualization_call('finish', False, 'Docking cancelled')
        result = Dock.Result()
        result.success = False
        result.error_code = Dock.Result.CANCELLED
        result.message = 'Docking cancelled'
        if goal_handle.is_active:
            goal_handle.canceled()
        return result

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def _rate_limit(value, previous, maximum_change):
        return max(previous - maximum_change, min(previous + maximum_change, value))

    def destroy_node(self):
        if rclpy.ok():
            self._stop_robot()
        self.action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DockingServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
