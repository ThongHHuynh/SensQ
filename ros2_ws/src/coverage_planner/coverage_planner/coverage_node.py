#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class Segment:
    def __init__(self, y, x1, x2):
        self.y = y
        self.x1 = x1
        self.x2 = x2
        self.visited = False

    def is_connected(self, other, spacing_px):
        return (self.x1 - spacing_px) <= other.x2 and (self.x2 + spacing_px) >= other.x1

class CoveragePlanner(Node):
    def __init__(self):
        super().__init__("coverage_planner")

        self.map_msg = None
        self.has_run = False

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("path_topic", "/coverage_path")
        self.declare_parameter("marker_topic", "/coverage_points")
        self.declare_parameter("clearance_m", 0.35)
        self.declare_parameter("spacing_m", 0.30)
        self.declare_parameter("min_segment_length_m", 0.50)
        self.declare_parameter("execute_coverage", False)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE

        map_topic = self.get_parameter("map_topic").value
        path_topic = self.get_parameter("path_topic").value
        marker_topic = self.get_parameter("marker_topic").value

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            map_qos
        )

        self.path_pub = self.create_publisher(Path, path_topic, map_qos)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, map_qos)

        self.latest_path = None
        self.latest_markers = None
        self.timer = self.create_timer(1.0, self.publish_latest)

        self.execute_coverage = self.get_parameter("execute_coverage").value
        self.navigator = BasicNavigator() if self.execute_coverage else None

        self.get_logger().info(f"Waiting for {map_topic}...")

    def map_callback(self, msg):
        if self.has_run:
            return

        self.has_run = True
        self.map_msg = msg
        self.get_logger().info("Map received.")
        self.destroy_subscription(self.map_sub)
        self.run_coverage()

    def run_coverage(self):
        msg = self.map_msg

        grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height,
            msg.info.width
        )

        free = (grid == 0).astype(np.uint8) * 255

        clearance_m = float(self.get_parameter("clearance_m").value)
        spacing_m = float(self.get_parameter("spacing_m").value)
        min_segment_length_m = float(self.get_parameter("min_segment_length_m").value)

        r = max(1, int(clearance_m / msg.info.resolution))

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * r + 1, 2 * r + 1)
        )

        safe_free = cv2.erode(free, kernel)

        waypoints_px = self.generate_lawnmower_path(
            safe_free,
            msg.info.resolution,
            spacing_m,
            min_segment_length_m
        )

        poses = []

        for x_px, y_px, yaw in waypoints_px:
            x, y = self.pixel_to_map(x_px, y_px, msg)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp.sec = 0
            pose.header.stamp.nanosec = 0

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.z = float(np.sin(yaw / 2.0))
            pose.pose.orientation.w = float(np.cos(yaw / 2.0))

            poses.append(pose)

        self.publish_path(poses)
        self.publish_markers(poses)

        self.get_logger().info(f"Published {len(poses)} coverage poses.")
        self.get_logger().info("RViz: add /coverage_path as Path.")
        self.get_logger().info("RViz: add /coverage_points as MarkerArray.")

        if len(poses) == 0:
            self.get_logger().warn("No coverage poses generated.")
            return

        if not self.execute_coverage:
            self.get_logger().info("Set execute_coverage:=true to send poses to Nav2.")
            return

        self.navigator.waitUntilNav2Active()
        
        key_poses = []
        if self.latest_path.poses:
            key_poses.append(self.latest_path.poses[0])
            for i in range(0, len(self.latest_path.poses) - 1):
                curr_z = self.latest_path.poses[i].pose.orientation.z
                next_z = self.latest_path.poses[i+1].pose.orientation.z
                if abs(curr_z - next_z) > 0.01:
                    key_poses.append(self.latest_path.poses[i])
            key_poses.append(self.latest_path.poses[-1])
            
        first_pose = key_poses[0]
        self.get_logger().info(f"Extracted {len(key_poses)} key poses. Navigating to start...")
        self.navigator.goToPose(first_pose)
        
        while not self.navigator.isTaskComplete():
            pass
            
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Reached start. Executing coverage path using NavigateThroughPoses...")
            now = self.navigator.get_clock().now().to_msg()
            for pose in key_poses:
                pose.header.stamp = now
            self.navigator.goThroughPoses(key_poses)
        else:
            self.get_logger().error("Failed to reach start of coverage path.")

    def generate_lawnmower_path(self, safe_free, resolution, spacing_m, min_segment_length_m):
        spacing_px = max(1, int(spacing_m / resolution))
        min_segment_length_px = max(1, int(min_segment_length_m / resolution))
        height, width = safe_free.shape

        segments = []
        for y in range(0, height, spacing_px):
            xs = np.where(safe_free[y, :] > 0)[0]
            if len(xs) == 0:
                continue
            segs = np.split(xs, np.where(np.diff(xs) > 1)[0] + 1)
            for seg in segs:
                if len(seg) >= min_segment_length_px:
                    segments.append(Segment(y, int(seg[0]), int(seg[-1])))

        if not segments:
            return []

        waypoints = []

        def add_segment_waypoints(seg, direction):
            if direction == 1:
                for x in range(seg.x1, seg.x2, spacing_px):
                    waypoints.append((x, seg.y, 0.0))
                waypoints.append((seg.x2, seg.y, 0.0))
                return seg.x2, seg.y
            else:
                for x in range(seg.x2, seg.x1, -spacing_px):
                    waypoints.append((x, seg.y, 3.14159))
                waypoints.append((seg.x1, seg.y, 3.14159))
                return seg.x1, seg.y

        def add_transition(curr_x, curr_y, next_seg, h_dir):
            """Add vertical-then-horizontal waypoints to avoid diagonal cuts."""
            if h_dir == 1:
                target_x = next_seg.x1
            else:
                target_x = next_seg.x2
            target_y = next_seg.y
            yaw = 1.5708 if target_y > curr_y else -1.5708
            waypoints.append((curr_x, target_y, yaw))
            if target_x != curr_x:
                move_yaw = 0.0 if target_x > curr_x else 3.14159
                waypoints.append((target_x, target_y, move_yaw))

        def get_next_connected(curr, target_y, curr_x):
            candidates = [s for s in segments if not s.visited and s.y == target_y and s.is_connected(curr, spacing_px)]
            if candidates:
                return min(candidates, key=lambda s: min(abs(s.x1 - curr_x), abs(s.x2 - curr_x)))
            return None

        curr_seg = segments[0]
        curr_seg.visited = True
        h_dir = 1
        v_dir = 1

        curr_x, curr_y = add_segment_waypoints(curr_seg, h_dir)

        while True:
            next_seg = get_next_connected(curr_seg, curr_seg.y + v_dir * spacing_px, curr_x)

            if next_seg is None:
                v_dir *= -1
                next_seg = get_next_connected(curr_seg, curr_seg.y + v_dir * spacing_px, curr_x)

            if next_seg is not None:
                h_dir *= -1
                add_transition(curr_x, curr_y, next_seg, h_dir)
                curr_seg = next_seg
                curr_seg.visited = True
                curr_x, curr_y = add_segment_waypoints(curr_seg, h_dir)
            else:
                unvisited = [s for s in segments if not s.visited]
                if not unvisited:
                    break

                def dist_to_seg(s):
                    return min((s.x1 - curr_x)**2 + (s.y - curr_y)**2, (s.x2 - curr_x)**2 + (s.y - curr_y)**2)

                curr_seg = min(unvisited, key=dist_to_seg)
                curr_seg.visited = True

                d1 = (curr_seg.x1 - curr_x)**2 + (curr_seg.y - curr_y)**2
                d2 = (curr_seg.x2 - curr_x)**2 + (curr_seg.y - curr_y)**2
                h_dir = 1 if d1 <= d2 else -1
                v_dir = 1
                curr_x, curr_y = add_segment_waypoints(curr_seg, h_dir)

        return waypoints

    def pixel_to_map(self, x_px, y_px, msg):
        resolution = msg.info.resolution
        origin = msg.info.origin
        local_x = (x_px + 0.5) * resolution
        local_y = (y_px + 0.5) * resolution

        yaw = self.quaternion_to_yaw(origin.orientation)
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        x = origin.position.x + local_x * cos_yaw - local_y * sin_yaw
        y = origin.position.y + local_x * sin_yaw + local_y * cos_yaw
        return x, y

    def quaternion_to_yaw(self, orientation):
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return float(np.arctan2(siny_cosp, cosy_cosp))

    def publish_path(self, poses):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp.sec = 0
        path.header.stamp.nanosec = 0
        path.poses = poses

        self.latest_path = path
        self.path_pub.publish(self.latest_path)

    def publish_markers(self, poses):
        marker_array = MarkerArray()

        # Delete all old markers first
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp.sec = 0
        delete_marker.header.stamp.nanosec = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Sphere list for all waypoints
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "coverage_waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.color.a = 1.0

        for pose in poses:
            marker.points.append(pose.pose.position)

        marker_array.markers.append(marker)

        # Numbered text labels at every Nth waypoint
        total = len(poses)
        label_step = max(1, total // 50)
        for i, pose in enumerate(poses):
            if i % label_step != 0 and i != total - 1:
                continue
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp.sec = 0
            text_marker.header.stamp.nanosec = 0
            text_marker.ns = "coverage_labels"
            text_marker.id = i + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = pose.pose
            text_marker.pose.position.z = 0.15
            text_marker.scale.z = 0.12
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.9
            text_marker.text = str(i)
            marker_array.markers.append(text_marker)

        self.latest_markers = marker_array
        self.marker_pub.publish(self.latest_markers)

    def publish_latest(self):
        if self.latest_path is not None:
            self.path_pub.publish(self.latest_path)
        if self.latest_markers is not None:
            self.marker_pub.publish(self.latest_markers)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
