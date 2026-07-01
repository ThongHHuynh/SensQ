#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

class MapProcessorNode(Node):
    def __init__(self):
        super().__init__('map_processor_node')

        self.declare_parameter("clearance_m", 0.3)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            qos_profile=map_qos
        )
        self.safe_map_pub = self.create_publisher(
            OccupancyGrid, 
            "/coverage/safe_map",
            qos_profile=map_qos
        )
        
        self.latest_safe_map = None
        self.timer = self.create_timer(1.0, self.publish_latest)
        
        self.get_logger().info("Map Processor Node has been started.")
        self.get_logger().info("Waiting for /map topic...")

    def map_callback(self, msg):
        clearance_m = self.get_parameter("clearance_m").value

        grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height,
            msg.info.width
        )
        free = (grid == 0).astype(np.uint8) * 255
        radius_px = int(clearance_m / msg.info.resolution)

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * radius_px + 1, 2 * radius_px + 1)
        )

        safe_free = cv2.erode(free, kernel)
        safe_grid = np.full_like(grid, 100)
        safe_grid[safe_free > 0] = 0

        safe_msg = OccupancyGrid()
        safe_msg.header = msg.header
        safe_msg.info = msg.info
        safe_msg.data = safe_grid.flatten().astype(int).tolist()
        
        self.latest_safe_map = safe_msg
        self.safe_map_pub.publish(self.latest_safe_map)
        self.get_logger().info("Generated /coverage/safe_map")

    def publish_latest(self):
        if self.latest_safe_map is not None:
            self.safe_map_pub.publish(self.latest_safe_map)

def main(args=None):
    rclpy.init(args=args)
    node = MapProcessorNode()
    rclpy.spin(node)
    rclpy.shutdown()