"""Priority velocity arbiter for navigation and docking commands."""

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class VelocityArbiter(Node):
    """Give fresh docking commands priority over navigation commands."""

    def __init__(self):
        super().__init__('velocity_arbiter')

        self.declare_parameter('nav_input_topic', '/cmd_vel')
        self.declare_parameter('dock_input_topic', '/cmd_vel_dock')
        self.declare_parameter('output_topic', '/cmd_vel_out')
        self.declare_parameter('nav_timeout', 0.5)
        self.declare_parameter('dock_timeout', 0.25)
        self.declare_parameter('publish_rate', 50.0)

        nav_topic = self.get_parameter('nav_input_topic').value
        dock_topic = self.get_parameter('dock_input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._nav_timeout = float(self.get_parameter('nav_timeout').value)
        self._dock_timeout = float(self.get_parameter('dock_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        if self._nav_timeout <= 0.0 or self._dock_timeout <= 0.0:
            raise ValueError('Velocity input timeouts must be positive')
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')

        self._lock = threading.Lock()
        self._nav_command = Twist()
        self._dock_command = Twist()
        self._nav_stamp = None
        self._dock_stamp = None

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, nav_topic, self._nav_callback, 10)
        self.create_subscription(Twist, dock_topic, self._dock_callback, 10)
        self.create_timer(1.0 / publish_rate, self._publish_selected)

        self.get_logger().info(
            f'Velocity arbitration: nav={nav_topic}, dock={dock_topic}, '
            f'output={output_topic}'
        )

    @staticmethod
    def _copy_twist(message):
        copied = Twist()
        copied.linear.x = message.linear.x
        copied.linear.y = message.linear.y
        copied.linear.z = message.linear.z
        copied.angular.x = message.angular.x
        copied.angular.y = message.angular.y
        copied.angular.z = message.angular.z
        return copied

    def _nav_callback(self, message):
        with self._lock:
            self._nav_command = self._copy_twist(message)
            self._nav_stamp = self.get_clock().now()

    def _dock_callback(self, message):
        with self._lock:
            self._dock_command = self._copy_twist(message)
            self._dock_stamp = self.get_clock().now()

    def _publish_selected(self):
        now = self.get_clock().now()
        with self._lock:
            dock_fresh = (
                self._dock_stamp is not None
                and (now - self._dock_stamp).nanoseconds / 1e9
                <= self._dock_timeout
            )
            nav_fresh = (
                self._nav_stamp is not None
                and (now - self._nav_stamp).nanoseconds / 1e9
                <= self._nav_timeout
            )

            if dock_fresh:
                command = self._copy_twist(self._dock_command)
            elif nav_fresh:
                command = self._copy_twist(self._nav_command)
            else:
                command = Twist()

        self._publisher.publish(command)


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = VelocityArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
