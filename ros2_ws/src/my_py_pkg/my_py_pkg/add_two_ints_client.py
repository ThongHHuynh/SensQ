#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("subs")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")

    def call_add_two_ints(self, a, b):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for service")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future  =self.client_.call_async(request)
        future.add_done_callback(self.callback_add_two_ints)

    def callback_add_two_ints(self,future):
        response = future.result()
        self.get_logger().info("Got response: " + str(response.sum))

def main(args = None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    node.call_add_two_ints(2,7)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()