#!/usr/bin/env python3

import rclpy
import numpy as np 

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

class Segment:
    def __init__(self, y, x1, x2):
        self.y = y
        self.x1 = x1
        self.x2 = x2
        self.visited = False

    def is_connected(self, other, spacing_px):
        return (self.x1 - spacing_px) <= other.x2 and (self.x2 + spacing_px) >= other.x1

class PathGeneratorNode(Node):
    def __init__(self):
        super().__init__('path_generator_node')

        self.declare_parameter("map_topic", "/coverage/safe_map")
        self.declare_parameter("path_topic", "/coverage/path")
        self.declare_parameter("spacing_m", 0.3)

        map_topic = self.get_parameter("map_topic").value
        path_topic = self.get_parameter("path_topic").value

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.safe_map_sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            qos_profile=map_qos
        )

        self.path_pub = self.create_publisher(
            Path,
            path_topic,
            qos_profile=map_qos
        )

        self.latest_path = None
        self.timer = self.create_timer(1.0, self.publish_latest)

        self.get_logger().info(f"Waiting for {map_topic}...")

    def map_callback(self, msg):
        spacing_m = self.get_parameter("spacing_m").value

        grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height,
            msg.info.width
        )

        safe_free = (grid == 0).astype(np.uint8) * 255

        waypoints = self.generate_lawnmower(
            safe_free, 
            msg.info.resolution,
            spacing_m
        )

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp.sec = 0
        path.header.stamp.nanosec = 0

        for x_px, y_px, yaw in waypoints:
            x, y = self.pixel_to_map(x_px, y_px, msg)
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = float(np.sin(yaw / 2.0))
            pose.pose.orientation.w = float(np.cos(yaw / 2.0))

            path.poses.append(pose)

        self.latest_path = path
        self.path_pub.publish(self.latest_path)

        self.get_logger().info(f"Generated path with {len(path.poses)} waypoints.")

    def publish_latest(self):
        if self.latest_path is not None:
            self.path_pub.publish(self.latest_path)

    def generate_lawnmower(self, safe_free, resolution, spacing_m):
        spacing_px = max(1, int(spacing_m / resolution))
        height, width = safe_free.shape
        
        segments = []
        for y in range(0, height, spacing_px):
            xs = np.where(safe_free[y, :] > 0)[0]
            if len(xs) == 0:
                continue
            segs = np.split(xs, np.where(np.diff(xs) > 1)[0] + 1)
            for seg in segs:
                if len(seg) >= 5:
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
            # Determine where the next row sweep will start
            if h_dir == 1:
                target_x = next_seg.x1
            else:
                target_x = next_seg.x2
            target_y = next_seg.y
            # Step 1: move vertically at curr_x to the next row's Y
            yaw = 1.5708 if target_y > curr_y else -1.5708  # pi/2 or -pi/2
            waypoints.append((curr_x, target_y, yaw))
            # Step 2: if the start of the next segment is not at curr_x,
            # move horizontally along the new row to reach it
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
                # Add L-shaped transition instead of diagonal jump
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
        x = msg.info.origin.position.x + x_px * msg.info.resolution
        y = msg.info.origin.position.y + y_px * msg.info.resolution
        return x, y
    
def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()