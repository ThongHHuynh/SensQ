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

from my_robot_docking.approach_controller import (
    ApproachConfig,
    ApproachState,
    CorridorApproachController,
)
from my_robot_docking.dock_database import DockDatabase, DockDatabaseError
from my_robot_docking.docking_geometry import (
    Pose2D,
    minimum_range_excluding_sector,
    normalize_angle,
    scan_sector_clearances,
    target_pose_from_tag,
    transform_pose,
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
        self._scan_lock = threading.Lock()
        self._scan = None
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

        self._log_database_warnings(self.database)
        self.get_logger().info(
            f'Docking server ready: action={self.dock_action}, '
            f'docks={self.database.ids()}'
        )

    def _log_database_warnings(self, database):
        for warning in database.warnings():
            self.get_logger().warning(warning)

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

        self._log_database_warnings(self.database)
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
            'cross_track_kp': 1.2,
            'yaw_kp': 0.6,
            'distance_tolerance': 0.03,
            'lateral_tolerance': 0.03,
            'yaw_tolerance': 0.08,
            'minimum_tag_distance': 0.25,
            'corridor_half_width': 0.12,
            'entry_margin': 0.05,
            'entry_yaw_tolerance': 0.25,
            'entry_position_tolerance': 0.06,
            'align_yaw_tolerance': 0.05,
            'overshoot_margin': 0.08,
            'min_linear_speed': 0.03,
            'min_angular_speed': 0.10,
            'approach_taper_distance': 0.25,
            'enter_drive_abort_angle': 0.60,
            'run_abort_yaw': 0.50,
            'dock_keepout_radius': 0.35,
            'entry_arc_step': 0.50,
            'max_corridor_replans': 3,
            'anchor_filter_alpha': 0.35,
            'anchor_samples': 5,
            'anchor_timeout': 1.5,
            'rotation_clearance_exclusion': 0.60,
            'dock_contact_distance': 0.05,
            'front_stop_distance': 0.18,
            'front_sector_half_angle': 0.35,
            'rear_stop_distance': 0.18,
            'rear_sector_half_angle': 0.35,
            'scan_forward_angle': math.pi,
            'max_reverse_distance': 1.0,
            'sensor_timeout': 0.5,
            'command_timeout': 0.25,
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
            'cross_track_kp',
            'yaw_kp',
            'distance_tolerance',
            'lateral_tolerance',
            'yaw_tolerance',
            'minimum_tag_distance',
            'corridor_half_width',
            'entry_margin',
            'entry_yaw_tolerance',
            'entry_position_tolerance',
            'align_yaw_tolerance',
            'overshoot_margin',
            'min_linear_speed',
            'min_angular_speed',
            'approach_taper_distance',
            'enter_drive_abort_angle',
            'run_abort_yaw',
            'dock_keepout_radius',
            'entry_arc_step',
            'max_corridor_replans',
            'anchor_filter_alpha',
            'anchor_samples',
            'anchor_timeout',
            'rotation_clearance_exclusion',
            'dock_contact_distance',
            'front_stop_distance',
            'front_sector_half_angle',
            'rear_stop_distance',
            'rear_sector_half_angle',
            'scan_forward_angle',
            'max_reverse_distance',
            'sensor_timeout',
            'command_timeout',
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
            'cross_track_kp',
            'yaw_kp',
            'distance_tolerance',
            'lateral_tolerance',
            'yaw_tolerance',
            'minimum_tag_distance',
            'corridor_half_width',
            'entry_yaw_tolerance',
            'entry_position_tolerance',
            'align_yaw_tolerance',
            'overshoot_margin',
            'min_linear_speed',
            'min_angular_speed',
            'approach_taper_distance',
            'enter_drive_abort_angle',
            'run_abort_yaw',
            'dock_keepout_radius',
            'entry_arc_step',
            'anchor_timeout',
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
        if self.min_linear_speed > self.max_linear_speed:
            raise ValueError('min_linear_speed cannot exceed max_linear_speed')
        if self.min_angular_speed > self.max_angular_speed:
            raise ValueError(
                'min_angular_speed cannot exceed max_angular_speed'
            )
        if self.align_yaw_tolerance > self.entry_yaw_tolerance:
            raise ValueError(
                'align_yaw_tolerance cannot exceed entry_yaw_tolerance'
            )
        if self.overshoot_margin <= self.distance_tolerance:
            raise ValueError('overshoot_margin must exceed distance_tolerance')
        if self.entry_margin < 0.0:
            raise ValueError('entry_margin cannot be negative')
        if self.dock_contact_distance < 0.0:
            raise ValueError('dock_contact_distance cannot be negative')
        if self.anchor_samples < 1:
            raise ValueError('anchor_samples must be at least one')
        if self.max_corridor_replans < 0:
            raise ValueError('max_corridor_replans cannot be negative')
        if not 0.0 < self.anchor_filter_alpha <= 1.0:
            raise ValueError('anchor_filter_alpha must be within (0, 1]')

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
        with self._scan_lock:
            self._scan = (
                tuple(message.ranges),
                float(message.angle_min),
                float(message.angle_increment),
                float(message.range_min),
                float(message.range_max),
            )
        self._last_scan_received_ns = self.get_clock().now().nanoseconds

    def _rotation_clearance(self, exclude_bearing=None):
        """Closest return that a rotation in place would sweep into.

        Rotating sweeps the whole footprint, so every bearing matters -- except
        the dock the robot is deliberately parked in front of, which would
        otherwise dominate the minimum and veto every turn near a station.
        """

        with self._scan_lock:
            scan = self._scan
        if scan is None:
            return math.inf

        ranges, angle_min, angle_increment, range_min, range_max = scan
        return minimum_range_excluding_sector(
            ranges,
            angle_min,
            angle_increment,
            range_min,
            range_max,
            self.scan_forward_angle,
            0.0 if exclude_bearing is None else exclude_bearing,
            0.0 if exclude_bearing is None
            else self.rotation_clearance_exclusion,
        )

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

        final_distance = (
            float(request.final_distance)
            if request.use_offset_override
            else dock.final_distance
        )
        contact_band = final_distance + self.dock_contact_distance
        approach_stop = (
            self.rear_stop_distance
            if dock.reverse_docking
            else self.front_stop_distance
        )
        if approach_stop > contact_band:
            self.get_logger().warning(
                f'Dock {dock.dock_id}: the directional stop at '
                f'{approach_stop:.3f} m fires before the dock face at '
                f'{contact_band:.3f} m is reached; raise '
                'dock_contact_distance or lower the stop distance'
            )

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
                    outcome, message = self._approach_tag(
                        goal_handle,
                        dock,
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

            return self._abort_result(
                goal_handle,
                Dock.Result.CONTROL_FAILED,
                'Docking ended unexpectedly',
            )
        except Exception as error:  # Keep the action from leaving motion active.
            self.get_logger().error(f'Docking failed: {error}')
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

            safety = self._search_safety_reason()
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

    def _search_safety_reason(self):
        """The tag sweep is a rotation in place, so it needs the same guard."""

        reason = self._sensor_safety_reason(require_scan=True)
        if reason is not None:
            return reason
        clearance = self._rotation_clearance()
        if clearance < self.rotation_stop_distance:
            return (
                f'Obstacle at {clearance:.3f} m is inside the '
                f'{self.rotation_stop_distance:.3f} m rotation stop distance'
            )
        return None

    def _retry_recovery_safety_reason(self, rotating):
        reason = self._sensor_safety_reason(require_scan=True)
        if reason is not None:
            return reason
        if rotating:
            clearance = self._rotation_clearance()
            if clearance < self.rotation_stop_distance:
                return (
                    f'Obstacle at {clearance:.3f} m is inside '
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

    def _approach_config(self):
        return ApproachConfig(
            corridor_half_width=self.corridor_half_width,
            entry_margin=self.entry_margin,
            entry_yaw_tolerance=self.entry_yaw_tolerance,
            entry_position_tolerance=self.entry_position_tolerance,
            align_yaw_tolerance=self.align_yaw_tolerance,
            distance_tolerance=self.distance_tolerance,
            lateral_tolerance=self.lateral_tolerance,
            yaw_tolerance=self.yaw_tolerance,
            overshoot_margin=self.overshoot_margin,
            max_linear_speed=self.max_linear_speed,
            min_linear_speed=self.min_linear_speed,
            max_angular_speed=self.max_angular_speed,
            min_angular_speed=self.min_angular_speed,
            distance_kp=self.distance_kp,
            heading_kp=self.heading_kp,
            cross_track_kp=self.cross_track_kp,
            yaw_kp=self.yaw_kp,
            approach_taper_distance=self.approach_taper_distance,
            enter_drive_abort_angle=self.enter_drive_abort_angle,
            run_abort_yaw=self.run_abort_yaw,
            dock_keepout_radius=self.dock_keepout_radius,
            entry_arc_step=self.entry_arc_step,
            max_corridor_replans=self.max_corridor_replans,
        )

    def _approach_tag(
        self,
        goal_handle,
        dock,
        final_distance,
        lateral_offset,
        yaw_offset,
        retries,
    ):
        """Run one corridor approach to the dock's final pose.

        The observation that ended the search is not reused: the approach
        re-samples the tag so the axis it plans from is an average rather than
        whichever single frame happened to break the search loop.
        """

        # Backing in enters the corridor at the staging distance so the blind
        # rear-first leg stays short; driving in enters at the predocking
        # distance, which leaves room to swing onto the axis.
        entry_distance = (
            dock.staging_distance
            if dock.reverse_docking
            else dock.predocking_distance
        )
        self._visualization_call(
            'set_stage',
            DockingStage.REVERSE_FINAL
            if dock.reverse_docking
            else DockingStage.FINAL_APPROACH,
        )
        return self._run_corridor_approach(
            goal_handle,
            dock,
            final_distance,
            entry_distance,
            lateral_offset,
            yaw_offset,
            retries,
            reverse=dock.reverse_docking,
            success_message=(
                'Reverse dock pose verified'
                if dock.reverse_docking
                else 'Dock pose verified'
            ),
            target_name='final docking pose',
        )

    def _run_corridor_approach(
        self,
        goal_handle,
        dock,
        target_distance,
        entry_distance,
        lateral_offset,
        yaw_offset,
        retries,
        reverse,
        success_message,
        target_name,
    ):
        """Drive onto the dock's approach axis and then down it.

        The controller works in the odometry frame against a latched tag pose
        that is refreshed whenever the tag is in view. Anchoring this way is
        what lets the robot turn away from the dock during entry: the camera
        looks forward over a limited field of view, so any manoeuvre that puts
        the robot on the axis necessarily loses sight of the tag part way.
        """

        anchor = self._anchor_from_tag(dock)
        if anchor is None:
            return ApproachOutcome.TAG_LOST, f'Lost tag {dock.tag_id}'

        robot_pose = self._latest_odom_pose()
        if robot_pose is None:
            return ApproachOutcome.SAFETY_STOP, 'Odometry pose is unavailable'

        controller = CorridorApproachController(
            self._approach_config(),
            target_distance,
            entry_distance,
            lateral_offset,
            yaw_offset,
            reverse=reverse,
        )
        controller.anchor(anchor)

        deadline = time.monotonic() + self.approach_timeout
        period = 1.0 / self.control_rate
        verification_started = None
        last_tag_seen = time.monotonic()
        reverse_travelled = 0.0
        previous_pose = robot_pose
        last_state = None

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._stop_robot()
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

            observation = self.tag_tracker.latest(dock.tag_id, dock.tag_frame)
            tag_fresh = observation is not None
            if tag_fresh:
                last_tag_seen = time.monotonic()
                anchor = self._blend_pose(
                    anchor,
                    transform_pose(
                        robot_pose,
                        Pose2D(
                            observation.x,
                            observation.y,
                            observation.normal_yaw,
                        ),
                    ),
                )
                controller.anchor(anchor)
            elif (
                controller.requires_tag
                and time.monotonic() - last_tag_seen
                >= self.tag_loss_grace_period
            ):
                self._stop_robot()
                return ApproachOutcome.TAG_LOST, f'Lost tag {dock.tag_id}'

            update = controller.update(robot_pose, tag_fresh)
            error = update.error

            if update.state != last_state:
                self._log_corridor_state(dock, update, reverse)
                self._visualization_call(
                    'set_stage',
                    self._corridor_stage(update.state, reverse),
                )
                last_state = update.state
            goal_pose = controller.goal_pose()
            if goal_pose is not None:
                self._visualization_call(
                    'update_live_target',
                    update.waypoint or goal_pose,
                    self._latest_odom_frame_id(),
                    robot_pose,
                )

            tag_distance = math.hypot(
                anchor.x - robot_pose.x,
                anchor.y - robot_pose.y,
            )
            safety = self._corridor_safety_reason(
                update.state,
                update.linear,
                tag_distance,
                target_distance,
                anchor,
                robot_pose,
            )
            if safety is not None:
                self._stop_robot()
                return ApproachOutcome.SAFETY_STOP, safety

            if update.linear < 0.0:
                reverse_travelled += math.hypot(
                    robot_pose.x - previous_pose.x,
                    robot_pose.y - previous_pose.y,
                )
                if reverse_travelled > self.max_reverse_distance:
                    self._stop_robot()
                    return (
                        ApproachOutcome.CONTROL_FAILED,
                        'Maximum reverse travel distance exceeded',
                    )
            previous_pose = robot_pose

            if update.overshot:
                self._stop_robot()
                return (
                    ApproachOutcome.CONTROL_FAILED,
                    f'Robot passed the configured {target_name}',
                )

            if update.reached:
                self._stop_robot()
                if verification_started is None:
                    verification_started = time.monotonic()
                self._publish_feedback(
                    goal_handle,
                    Dock.Feedback.VERIFYING,
                    error.along,
                    error.cross,
                    error.yaw,
                    retries,
                )
                if (
                    time.monotonic() - verification_started
                    >= self.verification_duration
                ):
                    return ApproachOutcome.SUCCEEDED, success_message
            else:
                verification_started = None
                self._publish_feedback(
                    goal_handle,
                    self._corridor_feedback_state(update.state),
                    error.along,
                    error.cross,
                    error.yaw,
                    retries,
                )
                command = Twist()
                command.linear.x = update.linear
                command.angular.z = update.angular
                self._publish_dock_command(command)

            time.sleep(period)

        self._stop_robot()
        return (
            ApproachOutcome.TIMED_OUT,
            f'Approach to {target_name} timed out',
        )

    def _anchor_from_tag(self, dock):
        """Average several observations into one odometry-frame tag pose.

        A single detection carries enough angular noise to tilt the whole
        approach axis, and the axis is what the entry legs are planned from,
        so the first anchor is worth a few control periods to get right.
        """

        poses = []
        stamps = set()
        deadline = time.monotonic() + self.anchor_timeout
        period = 1.0 / self.control_rate

        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and len(poses) < self.anchor_samples
        ):
            observation = self.tag_tracker.latest(dock.tag_id, dock.tag_frame)
            robot_pose = self._latest_odom_pose()
            if (
                observation is not None
                and robot_pose is not None
                and observation.stamp_seconds not in stamps
            ):
                stamps.add(observation.stamp_seconds)
                poses.append(
                    transform_pose(
                        robot_pose,
                        Pose2D(
                            observation.x,
                            observation.y,
                            observation.normal_yaw,
                        ),
                    )
                )
                if len(poses) >= self.anchor_samples:
                    break
            time.sleep(period)

        if not poses:
            return None
        return self._average_poses(poses)

    @staticmethod
    def _average_poses(poses):
        """Circular mean of tag poses, minus the single worst outlier."""

        def mean(samples):
            count = float(len(samples))
            return Pose2D(
                x=sum(pose.x for pose in samples) / count,
                y=sum(pose.y for pose in samples) / count,
                yaw=math.atan2(
                    sum(math.sin(pose.yaw) for pose in samples) / count,
                    sum(math.cos(pose.yaw) for pose in samples) / count,
                ),
            )

        centre = mean(poses)
        if len(poses) >= 4:
            worst = max(
                poses,
                key=lambda pose: math.hypot(
                    pose.x - centre.x,
                    pose.y - centre.y,
                ),
            )
            poses = [pose for pose in poses if pose is not worst]
            centre = mean(poses)
        return centre

    def _blend_pose(self, previous, measured):
        """Low-pass a new tag observation onto the latched anchor."""

        alpha = self.anchor_filter_alpha
        if previous is None or alpha >= 1.0:
            return measured
        return Pose2D(
            x=previous.x + alpha * (measured.x - previous.x),
            y=previous.y + alpha * (measured.y - previous.y),
            yaw=normalize_angle(
                previous.yaw
                + alpha * normalize_angle(measured.yaw - previous.yaw)
            ),
        )

    def _corridor_safety_reason(
        self,
        state,
        linear,
        tag_distance,
        target_distance,
        anchor,
        robot_pose,
    ):
        if tag_distance < self.minimum_tag_distance:
            return 'Tag is inside the minimum safe distance'

        if state in ApproachState.ROTATING:
            bearing = normalize_angle(
                math.atan2(
                    anchor.y - robot_pose.y,
                    anchor.x - robot_pose.x,
                )
                - robot_pose.yaw
            )
            clearance = self._rotation_clearance(exclude_bearing=bearing)
            if clearance < self.rotation_stop_distance:
                return (
                    f'Obstacle at {clearance:.3f} m is inside the '
                    f'{self.rotation_stop_distance:.3f} m rotation '
                    'stop distance'
                )
            return None

        if abs(linear) < 1e-6:
            return None

        # At the dock face the dock itself fills the sector being driven into,
        # so the directional stop would fire on the target. minimum_tag_distance
        # is the guard that remains inside that band.
        if tag_distance <= target_distance + self.dock_contact_distance:
            return None

        reverse = linear < 0.0
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

    @staticmethod
    def _corridor_stage(state, reverse):
        if state in (ApproachState.ENTER_TURN, ApproachState.ENTER_DRIVE):
            return DockingStage.STAGING_APPROACH
        if state == ApproachState.ALIGN:
            return DockingStage.PREDOCK_ALIGNMENT
        return (
            DockingStage.REVERSE_FINAL if reverse else DockingStage.FINAL_APPROACH
        )

    @staticmethod
    def _corridor_feedback_state(state):
        if state == ApproachState.ENTER_TURN:
            return Dock.Feedback.ALIGNING_PREDOCK
        if state == ApproachState.ENTER_DRIVE:
            return Dock.Feedback.MOVING_TO_STAGING
        if state == ApproachState.ALIGN:
            return Dock.Feedback.ALIGNING
        if state == ApproachState.REACHED:
            return Dock.Feedback.VERIFYING
        return Dock.Feedback.APPROACHING

    def _log_corridor_state(self, dock, update, reverse):
        mode = 'reverse' if reverse else 'forward'
        error = update.error
        self.get_logger().info(
            f'Docking {dock.dock_id} ({mode}): {update.state} '
            f'along={error.along:.3f} cross={error.cross:.3f} '
            f'yaw={error.yaw:.3f}'
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
