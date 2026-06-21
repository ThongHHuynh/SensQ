import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_listener")
        self.subscriber_ = self.create_subscription(Int64, "number", self.message_callback,10)
        self.get_logger().info("Number listener node started")

        self.publisher_ = self.create_publisher(Int64, "number_verified", 10)

        self.timer_ = self.create_timer(1.0, self.message_verified)
        
    def message_callback(self,msg):
        self.last_received_number = msg.data
        self.get_logger().info(f"Received: {self.last_received_number}")

    def message_verified(self):
        msg = Int64()
        msg.data = self.last_received_number
        self.publisher_.publish(msg)
        self.get_logger().info(f"Verified and republished data: {msg.data}")
        


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()