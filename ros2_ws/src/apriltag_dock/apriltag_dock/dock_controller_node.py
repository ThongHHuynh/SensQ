"""
Dock Controller Node
====================
State-machine-based precision docking controller.  Uses TF lookups to
the target AprilTag frame and publishes velocity commands on ``/cmd_vel``
to drive the robot directly in front of the tag.

States:  IDLE → SEARCHING → APPROACHING → ALIGNING → DOCKED
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros


class DockState:
    """Simple enum-like constants for the docking state machine."""
    IDLE = 'IDLE'
    SEARCHING = 'SEARCHING'
    APPROACHING = 'APPROACHING'
    ALIGNING = 'ALIGNING'
    DOCKED = 'DOCKED'


class DockControllerNode(Node):
    """Precision docking controller using AprilTag TF."""

    def __init__(self):
        super().__init__('dock_controller_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('dock_distance', 0.35)
        self.declare_parameter('min_safe_distance', 0.25)
        self.declare_parameter('approach_speed', 0.10)
        self.declare_parameter('search_angular_speed', 0.3)
        self.declare_parameter('lateral_kp', 0.8)
        self.declare_parameter('approach_kp', 0.4)
        self.declare_parameter('alignment_tolerance_m', 0.02)
        self.declare_parameter('alignment_tolerance_yaw', 0.05)
        self.declare_parameter('dock_frame', 'base_footprint')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('tag_lost_timeout', 3.0)
        self.declare_parameter('align_transition_distance', 0.50)

        self._load_params()

        # ── State ───────────────────────────────────────────────────────
        self.state = DockState.IDLE
        self._tag_lost_since = None  # monotonic time when tag was last seen

        # ── TF ──────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publishers ──────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/dock/status', 10)

        # ── Services ────────────────────────────────────────────────────
        self.create_service(Trigger, '/dock/start', self._start_cb)
        self.create_service(Trigger, '/dock/cancel', self._cancel_cb)

        # ── Control loop timer ──────────────────────────────────────────
        period = 1.0 / self.control_rate
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'Dock controller ready  '
            f'target=tag_{self.target_tag_id}  '
            f'dock_dist={self.dock_distance:.2f} m'
        )

    # ─────────────────────────────────────────────────────────────────────
    # Parameter helpers
    # ─────────────────────────────────────────────────────────────────────
    def _load_params(self):
        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.dock_distance = self.get_parameter('dock_distance').value
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.search_angular_speed = self.get_parameter('search_angular_speed').value
        self.lateral_kp = self.get_parameter('lateral_kp').value
        self.approach_kp = self.get_parameter('approach_kp').value
        self.align_tol_m = self.get_parameter('alignment_tolerance_m').value
        self.align_tol_yaw = self.get_parameter('alignment_tolerance_yaw').value
        self.dock_frame = self.get_parameter('dock_frame').value
        self.control_rate = self.get_parameter('control_rate').value
        self.tag_lost_timeout = self.get_parameter('tag_lost_timeout').value
        self.align_transition_dist = self.get_parameter('align_transition_distance').value

    # ─────────────────────────────────────────────────────────────────────
    # Service callbacks
    # ─────────────────────────────────────────────────────────────────────
    def _start_cb(self, request, response):
        if self.state != DockState.IDLE:
            response.success = False
            response.message = f'Cannot start: already in state {self.state}'
            self.get_logger().warn(response.message)
            return response

        self._load_params()  # pick up any dynamic changes
        self.state = DockState.SEARCHING
        self._tag_lost_since = None
        response.success = True
        response.message = (
            f'Docking started — searching for tag_{self.target_tag_id}'
        )
        self.get_logger().info(response.message)
        return response

    def _cancel_cb(self, request, response):
        prev = self.state
        self.state = DockState.IDLE
        self._stop_robot()
        response.success = True
        response.message = f'Docking cancelled (was {prev})'
        self.get_logger().info(response.message)
        return response

    # ─────────────────────────────────────────────────────────────────────
    # Control loop
    # ─────────────────────────────────────────────────────────────────────
    def _control_loop(self):
        # Always publish status
        status_msg = String()
        status_msg.data = self.state
        self.status_pub.publish(status_msg)

        if self.state == DockState.IDLE or self.state == DockState.DOCKED:
            return

        # Try to look up the tag transform
        tag_frame = f'tag_{self.target_tag_id}'
        try:
            transform = self.tf_buffer.lookup_transform(
                self.dock_frame, tag_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0),
            )
            self._tag_lost_since = None  # tag is visible
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            transform = None
            now = self.get_clock().now().nanoseconds / 1e9
            if self._tag_lost_since is None:
                self._tag_lost_since = now

        # ── Emergency stop if too close (any active state) ──────────────
        if transform is not None and self.state in (
            DockState.APPROACHING, DockState.ALIGNING
        ):
            t = transform.transform.translation
            distance = math.sqrt(t.x ** 2 + t.y ** 2)
            if distance < self.min_safe_distance:
                self.get_logger().warn(
                    f'EMERGENCY STOP — too close! dist={distance:.3f} m '
                    f'(min_safe={self.min_safe_distance:.2f} m)'
                )
                self._stop_robot()
                self.state = DockState.DOCKED
                return

        # ── State transitions & commands ────────────────────────────────
        if self.state == DockState.SEARCHING:
            self._handle_searching(transform)
        elif self.state == DockState.APPROACHING:
            self._handle_approaching(transform)
        elif self.state == DockState.ALIGNING:
            self._handle_aligning(transform)

    # ─────────────────────────────────────────────────────────────────────
    # State handlers
    # ─────────────────────────────────────────────────────────────────────
    def _handle_searching(self, transform):
        """Rotate in place until the tag is found."""
        if transform is not None:
            self.get_logger().info('Tag found — switching to APPROACHING')
            self.state = DockState.APPROACHING
            return

        cmd = Twist()
        cmd.angular.z = self.search_angular_speed
        self.cmd_vel_pub.publish(cmd)

    def _handle_approaching(self, transform):
        """Drive toward the tag while keeping it centred."""
        if transform is None:
            if self._tag_lost_too_long():
                self.get_logger().warn(
                    'Tag lost for too long — returning to SEARCHING'
                )
                self.state = DockState.SEARCHING
            else:
                self._stop_robot()
            return

        t = transform.transform.translation
        distance = math.sqrt(t.x ** 2 + t.y ** 2)
        lateral_error = t.y  # positive = tag is to the left
        yaw_error = math.atan2(t.y, t.x)

        self.get_logger().info(
            f'APPROACHING  dist={distance:.3f} m  '
            f'lateral={lateral_error:.3f} m  '
            f'yaw={math.degrees(yaw_error):.1f}°',
            throttle_duration_sec=0.5,
        )

        # Transition to ALIGNING when close enough
        if distance < self.align_transition_dist:
            self.get_logger().info(
                f'Close range ({distance:.2f} m) — switching to ALIGNING'
            )
            self.state = DockState.ALIGNING
            return

        # Proportional forward speed with deceleration ramp
        remaining = distance - self.dock_distance
        fwd_speed = self.approach_kp * remaining
        fwd_speed = self._clamp(fwd_speed, 0.0, self.approach_speed)

        # Proportional angular correction (reduce steering at close range)
        steer_scale = min(1.0, distance / 1.0)  # fade steering as we close in
        ang_speed = self.lateral_kp * yaw_error * steer_scale
        ang_speed = self._clamp(ang_speed, -0.4, 0.4)

        cmd = Twist()
        cmd.linear.x = fwd_speed
        cmd.angular.z = ang_speed
        self.cmd_vel_pub.publish(cmd)

    def _handle_aligning(self, transform):
        """Fine alignment at close range before declaring DOCKED."""
        if transform is None:
            if self._tag_lost_too_long():
                self.get_logger().warn(
                    'Tag lost during alignment — returning to SEARCHING'
                )
                self.state = DockState.SEARCHING
            else:
                self._stop_robot()
            return

        t = transform.transform.translation
        distance = math.sqrt(t.x ** 2 + t.y ** 2)
        lateral_error = abs(t.y)
        yaw_error = math.atan2(t.y, t.x)

        self.get_logger().info(
            f'ALIGNING  dist={distance:.3f} m  '
            f'lateral={lateral_error:.3f} m  '
            f'yaw={math.degrees(yaw_error):.1f}°',
            throttle_duration_sec=0.5,
        )

        # Check docked conditions
        is_close = distance <= self.dock_distance
        is_centred = lateral_error <= self.align_tol_m
        is_aligned = abs(yaw_error) <= self.align_tol_yaw

        if is_close and is_centred and is_aligned:
            self.get_logger().info(
                f'DOCKED!  distance={distance:.3f} m  '
                f'lateral={lateral_error:.3f} m  '
                f'yaw={math.degrees(yaw_error):.1f}°'
            )
            self.state = DockState.DOCKED
            self._stop_robot()
            return

        # If drifted too far, go back to approaching
        if distance > self.align_transition_dist + 0.10:
            self.state = DockState.APPROACHING
            return

        # Fine corrections — very slow and gentle
        remaining = distance - self.dock_distance
        fine_max = 0.4 * self.approach_speed  # cap at 40% of approach speed
        fwd = self.approach_kp * remaining
        fwd = self._clamp(fwd, 0.0, fine_max)

        ang = 0.6 * self.lateral_kp * yaw_error  # softer steering
        ang = self._clamp(ang, -0.2, 0.2)

        cmd = Twist()
        cmd.linear.x = fwd
        cmd.angular.z = ang
        self.cmd_vel_pub.publish(cmd)

    # ─────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────
    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def _tag_lost_too_long(self) -> bool:
        if self._tag_lost_since is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self._tag_lost_since) > self.tag_lost_timeout

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))


def main(args=None):
    rclpy.init(args=args)
    node = DockControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
