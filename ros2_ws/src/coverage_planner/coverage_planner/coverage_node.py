#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        self.map_msg = None

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/coverage_path',
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/coverage_points',
            10
        )

        self.navigator = BasicNavigator()

        self.get_logger().info("Waiting for /map...")

    def map_callback(self, msg):
        self.map_msg = msg
        self.get_logger().info("Map received.")

        self.destroy_subscription(self.map_sub)

        self.run_coverage()

    def run_coverage(self):
        msg = self.map_msg

        grid = np.array(msg.data).reshape(
            msg.info.height,
            msg.info.width
        )

        free = (grid == 0).astype(np.uint8) * 255

        clearance_m = 0.30
        r = int(clearance_m / msg.info.resolution)

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * r + 1, 2 * r + 1)
        )

        safe_free = cv2.erode(free, kernel)

        waypoints_px = self.generate_lawnmower_path(
            safe_free,
            msg.info.resolution,
            spacing_m=0.30
        )

        poses = []

        for x_px, y_px, yaw in waypoints_px:
            x, y = self.pixel_to_map(x_px, y_px, msg)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.z = np.sin(yaw / 2.0)
            pose.pose.orientation.w = np.cos(yaw / 2.0)

            poses.append(pose)

        self.publish_path(poses)
        self.publish_markers(poses)

        self.get_logger().info(f"Published {len(poses)} coverage poses.")
        self.get_logger().info("RViz topics: /coverage_path and /coverage_points")

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
                    waypoints.append((x2, y, 3.14159))
                    waypoints.append((x1, y, 3.14159))

                direction *= -1

        return waypoints

    def pixel_to_map(self, x_px, y_px, msg):
        x = msg.info.origin.position.x + x_px * msg.info.resolution
        y = msg.info.origin.position.y + y_px * msg.info.resolution
        return x, y

    def publish_path(self, poses):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses

        self.path_pub.publish(path)

    def publish_markers(self, poses):
        marker_array = MarkerArray()

        for i, pose in enumerate(poses):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = "coverage_waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose = pose.pose

            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.08

            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()