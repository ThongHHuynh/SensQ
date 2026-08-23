"""Reverse-and-turn undocking action server.

Backs the robot off the dock face, then rotates in place to face away from
it, publishing on the same ``cmd_vel_dock`` input the velocity arbiter
already gives priority over navigation commands.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from my_robot_docking.dock_database import DockDatabase, DockDatabaseError
from my_robot_docking.docking_geometry import (
    Pose2D,
    minimum_range_excluding_sector,
    normalize_angle,
    scan_sector_clearances,
)
from my_robot_docking_msgs.action import Undock


class UndockOutcome:
    SUCCEEDED = 'succeeded'
    SAFETY_STOP = 'safety_stop'
    CONTROL_FAILED = 'control_failed'
    CANCELLED = 'cancelled'


class UndockServer(Node):
    """Reverse off the dock face and rotate to face away from it."""

    def __init__(self):
        super().__init__('undock_server')
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

        self._odom_lock = threading.Lock()
        self._odom_pose = None
        self._last_odom_received_ns = None
        self._scan_lock = threading.Lock()
        self._scan = None
        self._last_scan_received_ns = None
        self._rear_clearance = math.inf
        self._last_command = Twist()
        self._last_command_time = time.monotonic()

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.marker_publisher = self.create_publisher(
            Marker,
            self.visualization_topic,
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

        self.action_server = ActionServer(
            self,
            Undock,
            self.undock_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self.callback_group,
        )

        self._goal_lock = threading.Lock()
        self._busy = False
        self._active_goal = None

        self.get_logger().info(
            f'Undock server ready: action={self.undock_action}, '
            f'docks={self.database.ids()}'
        )

    def _declare_parameters(self):
        declarations = {
            'dock_database_file': '',
            'undock_action': '/undock',
            'odom_topic': '/odometry/filtered',
            'scan_topic': '/scan',
            'cmd_vel_topic': '/cmd_vel_dock',
            'base_frame': 'base_footprint',
            'control_rate': 20.0,
            'legacy_predocking_offset': 0.5,
            'backup_distance': 0.30,
            'backup_speed': 0.10,
            'reversing_timeout': 15.0,
            'clearing_settle_duration': 0.30,
            'rotation_speed': 0.35,
            'heading_kp': 0.80,
            'yaw_tolerance': 0.08,
            'rotating_timeout': 15.0,
            'max_linear_acceleration': 0.20,
            'max_angular_acceleration': 0.80,
            'rear_stop_distance': 0.18,
            'rear_sector_half_angle': 0.35,
            'rotation_stop_distance': 0.22,
            'scan_forward_angle': math.pi,
            'sensor_timeout': 0.50,
            'command_timeout': 0.25,
            'require_fresh_odometry': True,
            'require_fresh_scan': True,
            'visualization_enabled': True,
            'visualization_topic': '/docking/markers',
        }
        for name, default in declarations.items():
            self.declare_parameter(name, default)

    def _load_parameters(self):
        for name in (
            'dock_database_file',
            'undock_action',
            'odom_topic',
            'scan_topic',
            'cmd_vel_topic',
            'base_frame',
            'control_rate',
            'legacy_predocking_offset',
            'backup_distance',
            'backup_speed',
            'reversing_timeout',
            'clearing_settle_duration',
            'rotation_speed',
            'heading_kp',
            'yaw_tolerance',
            'rotating_timeout',
            'max_linear_acceleration',
            'max_angular_acceleration',
            'rear_stop_distance',
            'rear_sector_half_angle',
            'rotation_stop_distance',
            'scan_forward_angle',
            'sensor_timeout',
            'command_timeout',
            'require_fresh_odometry',
            'require_fresh_scan',
            'visualization_enabled',
            'visualization_topic',
        ):
            setattr(self, name, self.get_parameter(name).value)

    def _validate_parameters(self):
        positive = (
            'control_rate',
            'legacy_predocking_offset',
            'backup_distance',
            'backup_speed',
            'reversing_timeout',
            'clearing_settle_duration',
            'rotation_speed',
            'heading_kp',
            'yaw_tolerance',
            'rotating_timeout',
            'max_linear_acceleration',
            'max_angular_acceleration',
            'rear_stop_distance',
            'rear_sector_half_angle',
            'rotation_stop_distance',
            'sensor_timeout',
            'command_timeout',
        )
        invalid = [name for name in positive if getattr(self, name) <= 0.0]
        if invalid:
            raise ValueError(
                f'Undock parameters must be positive: {", ".join(invalid)}'
            )
        if not math.isfinite(self.scan_forward_angle):
            raise ValueError('scan_forward_angle must be finite')

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
        self._last_odom_received_ns = self.get_clock().now().nanoseconds

    def _scan_callback(self, message):
        _front, rear = scan_sector_clearances(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
            self.scan_forward_angle,
            self.rear_sector_half_angle,
            self.rear_sector_half_angle,
        )
        self._rear_clearance = rear
        with self._scan_lock:
            self._scan = (
                tuple(message.ranges),
                float(message.angle_min),
                float(message.angle_increment),
                float(message.range_min),
                float(message.range_max),
            )
        self._last_scan_received_ns = self.get_clock().now().nanoseconds

    def _latest_odom_pose(self):
        with self._odom_lock:
            return self._odom_pose

    def _rotation_clearance(self):
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
            0.0,
            0.0,
        )

    def _sensor_safety_reason(self):
        now = self.get_clock().now().nanoseconds
        timeout_ns = int(self.sensor_timeout * 1e9)
        if self.require_fresh_odometry and (
            self._last_odom_received_ns is None
            or now - self._last_odom_received_ns > timeout_ns
        ):
            return 'Odometry is missing or stale'
        if self.require_fresh_scan and (
            self._last_scan_received_ns is None
            or now - self._last_scan_received_ns > timeout_ns
        ):
            return 'Laser scan is missing or stale'
        return None

    def _goal_callback(self, request):
        try:
            self.database.get(request.dock_id)
        except DockDatabaseError as error:
            self.get_logger().warning(str(error))
            return GoalResponse.REJECT

        with self._goal_lock:
            if self._busy:
                self.get_logger().warning('Undock server is already busy')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self._stop_robot()
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self._active_goal = goal_handle
        result = Undock.Result()
        request = goal_handle.request

        try:
            dock = self.database.get(request.dock_id)
        except DockDatabaseError as error:
            with self._goal_lock:
                self._busy = False
            self._active_goal = None
            return self._abort_result(
                goal_handle,
                result,
                Undock.Result.UNKNOWN_DOCK,
                str(error),
            )

        try:
            start_pose = self._latest_odom_pose()
            if start_pose is None:
                return self._abort_result(
                    goal_handle,
                    result,
                    Undock.Result.CONTROL_FAILED,
                    'Odometry pose is unavailable',
                )

            self._publish_markers(dock, start_pose)

            outcome, message, distance_cleared = self._reverse(
                goal_handle,
                start_pose,
            )
            if outcome != UndockOutcome.SUCCEEDED:
                return self._finish(goal_handle, result, outcome, message)

            self._stop_robot()
            self._publish_feedback(
                goal_handle,
                Undock.Feedback.CLEARING,
                distance_cleared,
            )
            time.sleep(self.clearing_settle_duration)

            # Rotation target is odom-relative (start heading + pi) rather
            # than the database's approach_yaw, since only odometry -- not
            # map localization -- is guaranteed to be available here.
            target_yaw = normalize_angle(start_pose.yaw + math.pi)
            outcome, message = self._rotate(
                goal_handle,
                target_yaw,
                distance_cleared,
            )
            if outcome != UndockOutcome.SUCCEEDED:
                return self._finish(goal_handle, result, outcome, message)

            self._stop_robot()
            self._publish_feedback(
                goal_handle,
                Undock.Feedback.COMPLETE,
                distance_cleared,
            )
            result.success = True
            result.error_code = Undock.Result.NONE
            result.message = f'Undocked from {dock.dock_id}'
            goal_handle.succeed()
            return result
        except Exception as error:  # Keep the action from leaving motion active.
            self.get_logger().error(f'Undocking failed: {error}')
            return self._abort_result(
                goal_handle,
                result,
                Undock.Result.CONTROL_FAILED,
                str(error),
            )
        finally:
            self._stop_robot()
            with self._goal_lock:
                self._busy = False
            self._active_goal = None

    def _reverse(self, goal_handle, start_pose):
        self._publish_feedback(goal_handle, Undock.Feedback.REVERSING, 0.0)
        period = 1.0 / self.control_rate
        deadline = time.monotonic() + self.reversing_timeout

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                return UndockOutcome.CANCELLED, 'Undocking cancelled', 0.0

            sensor_reason = self._sensor_safety_reason()
            if sensor_reason is not None:
                self._stop_robot()
                return UndockOutcome.SAFETY_STOP, sensor_reason, 0.0
            if self._rear_clearance < self.rear_stop_distance:
                self._stop_robot()
                return (
                    UndockOutcome.SAFETY_STOP,
                    f'Rear obstacle at {self._rear_clearance:.3f} m is inside '
                    f'the {self.rear_stop_distance:.3f} m stop distance',
                    0.0,
                )

            current_pose = self._latest_odom_pose()
            if current_pose is None:
                self._stop_robot()
                return (
                    UndockOutcome.SAFETY_STOP,
                    'Odometry pose is unavailable',
                    0.0,
                )

            travelled = math.hypot(
                current_pose.x - start_pose.x,
                current_pose.y - start_pose.y,
            )
            if travelled >= self.backup_distance:
                self._stop_robot()
                return UndockOutcome.SUCCEEDED, 'Backed clear of dock', travelled

            self._publish_feedback(
                goal_handle,
                Undock.Feedback.REVERSING,
                travelled,
            )
            command = Twist()
            command.linear.x = -self.backup_speed
            self._publish_command(command)
            time.sleep(period)

        self._stop_robot()
        return UndockOutcome.CONTROL_FAILED, 'Reversing off the dock timed out', 0.0

    def _rotate(self, goal_handle, target_yaw, distance_cleared):
        self._publish_feedback(
            goal_handle,
            Undock.Feedback.ROTATING,
            distance_cleared,
        )
        period = 1.0 / self.control_rate
        deadline = time.monotonic() + self.rotating_timeout

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                return UndockOutcome.CANCELLED, 'Undocking cancelled'

            sensor_reason = self._sensor_safety_reason()
            if sensor_reason is not None:
                self._stop_robot()
                return UndockOutcome.SAFETY_STOP, sensor_reason
            clearance = self._rotation_clearance()
            if clearance < self.rotation_stop_distance:
                self._stop_robot()
                return (
                    UndockOutcome.SAFETY_STOP,
                    f'Obstacle at {clearance:.3f} m is inside the '
                    f'{self.rotation_stop_distance:.3f} m rotation stop distance',
                )

            current_pose = self._latest_odom_pose()
            if current_pose is None:
                self._stop_robot()
                return UndockOutcome.SAFETY_STOP, 'Odometry pose is unavailable'

            yaw_error = normalize_angle(target_yaw - current_pose.yaw)
            if abs(yaw_error) <= self.yaw_tolerance:
                self._stop_robot()
                return UndockOutcome.SUCCEEDED, 'Facing away from dock'

            self._publish_feedback(
                goal_handle,
                Undock.Feedback.ROTATING,
                distance_cleared,
            )
            command = Twist()
            command.angular.z = self._clamp(
                self.heading_kp * yaw_error,
                -self.rotation_speed,
                self.rotation_speed,
            )
            self._publish_command(command)
            time.sleep(period)

        self._stop_robot()
        return UndockOutcome.CONTROL_FAILED, 'Rotating away from dock timed out'

    def _finish(self, goal_handle, result, outcome, message):
        error_code = {
            UndockOutcome.SAFETY_STOP: Undock.Result.SAFETY_STOP,
            UndockOutcome.CANCELLED: Undock.Result.CANCELLED,
            UndockOutcome.CONTROL_FAILED: Undock.Result.CONTROL_FAILED,
        }.get(outcome, Undock.Result.CONTROL_FAILED)
        if outcome == UndockOutcome.CANCELLED:
            return self._cancel_result(goal_handle, result, message)
        return self._abort_result(goal_handle, result, error_code, message)

    def _abort_result(self, goal_handle, result, error_code, message):
        self._stop_robot()
        result.success = False
        result.error_code = int(error_code)
        result.message = message
        if goal_handle.is_active:
            goal_handle.abort()
        return result

    def _cancel_result(self, goal_handle, result, message):
        self._stop_robot()
        result.success = False
        result.error_code = Undock.Result.CANCELLED
        result.message = message
        if goal_handle.is_active:
            goal_handle.canceled()
        return result

    def _publish_feedback(self, goal_handle, state, distance_cleared):
        feedback = Undock.Feedback()
        feedback.state = int(state)
        feedback.distance_cleared = float(distance_cleared)
        goal_handle.publish_feedback(feedback)

    def _publish_command(self, command):
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
        self.cmd_vel_publisher.publish(limited)
        self._last_command = limited
        self._last_command_time = now

    def _stop_robot(self):
        stop = Twist()
        self.cmd_vel_publisher.publish(stop)
        self._last_command = stop
        self._last_command_time = time.monotonic()

    def _publish_markers(self, dock, start_pose):
        if not self.visualization_enabled:
            return
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'undock'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.scale.y = 0.05
        marker.scale.z = 0.0
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.color.a = 1.0
        target_yaw = normalize_angle(start_pose.yaw + math.pi)
        start_point = Point(x=start_pose.x, y=start_pose.y, z=0.05)
        end_point = Point(
            x=start_pose.x + self.backup_distance * math.cos(target_yaw),
            y=start_pose.y + self.backup_distance * math.sin(target_yaw),
            z=0.05,
        )
        marker.points = [start_point, end_point]
        self.marker_publisher.publish(marker)
        self.get_logger().info(
            f'Undocking from {dock.dock_id} (approach_yaw='
            f'{dock.approach_yaw:.3f} rad); target heading '
            f'{target_yaw:.3f} rad'
        )

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
    node = UndockServer()
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
