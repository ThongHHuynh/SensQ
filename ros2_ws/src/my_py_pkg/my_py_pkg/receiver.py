#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("subs")
        Node.__init__(self,"subs")
        self.subscriber_ = self.create_subscription(String, "robot_news",self.subs_news,10)
        self.get_logger().info("Robot news has started")

    def subs_news(self,msg):
        self.get_logger().info(f'I heard {msg.data}')
def main(args = None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()