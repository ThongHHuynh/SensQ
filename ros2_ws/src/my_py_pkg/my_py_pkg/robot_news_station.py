#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.publishers_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(1, self.publish_news)
        self.get_logger().info("Robot news has started")
        self.counter_ = 0

    def publish_news(self):
        msg = String()
        msg.data = "Hello"
        self.publishers_.publish(msg)
        self.get_logger().info(f"Published time{self.counter_} with info: {msg.data}")
        self.counter_ +=1


def main(args = None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()