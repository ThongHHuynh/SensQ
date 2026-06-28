#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        self.map_msg = None
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.navigator = BasicNavigator()
        self.get_logger().info("Waiting for /map...")

    def map_callback(self, msg):
        self.map_msg = msg
        self.get_logger().info("Map received.")
        self.destroy_subscription(self.sub)
        self.run_coverage()

    def run_coverage(self):
        msg = self.map_msg

        grid = np.array(msg.data).reshape(
            msg.info.height,
            msg.info.width
        )

        free = (grid == 0).astype(np.uint8) * 255

        # shrink free space to keep robot away from walls
        clearance_m = 0.25
        r = int(clearance_m / msg.info.resolution)

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * r + 1, 2 * r + 1)
        )

        safe_free = cv2.erode(free, kernel)

        waypoints_px = self.generate_lawnmower_path(
            safe_free,
            msg.info.resolution,
            spacing_m=0.25
        )

        poses = []

        for x_px, y_px, yaw in waypoints_px:
            x, y = self.pixel_to_map(x_px, y_px, msg)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = np.sin(yaw / 2.0)
            pose.pose.orientation.w = np.cos(yaw / 2.0)

            poses.append(pose)

        self.get_logger().info(f"Generated {len(poses)} coverage poses.")

        self.navigator.waitUntilNav2Active()
        self.navigator.goThroughPoses(poses)

    def generate_lawnmower_path(self, safe_free, resolution, spacing_m):
        spacing_px = max(1, int(spacing_m / resolution))
        height, width = safe_free.shape

        waypoints = []
        direction = 1

        for y in range(0, height, spacing_px):
            xs = np.where(safe_free[y, :] > 0)[0]

            if len(xs) == 0:
                continue

            segments = np.split(xs, np.where(np.diff(xs) > 1)[0] + 1)

            for seg in segments:
                if len(seg) < 10:
                    continue

                x1 = int(seg[0])
                x2 = int(seg[-1])

                if direction == 1:
                    waypoints.append((x1, y, 0.0))
                    waypoints.append((x2, y, 0.0))
                else:
                    waypoints.append((x2, y, 3.14))
                    waypoints.append((x1, y, 3.14))

                direction *= -1

        return waypoints

    def pixel_to_map(self, x_px, y_px, msg):
        x = msg.info.origin.position.x + x_px * msg.info.resolution
        y = msg.info.origin.position.y + y_px * msg.info.resolution
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()