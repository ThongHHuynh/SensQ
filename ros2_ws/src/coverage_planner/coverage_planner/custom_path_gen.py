#!/usr/bin/env python3

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


@dataclass
class Segment:
    y: int
    x1: int
    x2: int
    cell_id: int


@dataclass
class Cell:
    id: int
    segments: list


@dataclass
class Waypoint:
    x: int
    y: int
    yaw: float


class CustomPathGenerator(Node):
    def __init__(self):
        super().__init__("custom_path_generator")

        self.declare_parameter("map_topic", "/coverage/safe_map")
        self.declare_parameter("path_topic", "/coverage/path")
        self.declare_parameter("spacing_m", 0.30)
        self.declare_parameter("min_segment_length_m", 0.20)
        self.declare_parameter("validation_step_px", 2)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.path_pub = self.create_publisher(
            Path,
            self.get_parameter("path_topic").value,
            qos,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.map_callback,
            qos,
        )

        self.latest_path = None
        self.timer = self.create_timer(1.0, self.publish_latest)

    def map_callback(self, msg):
        grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height,
            msg.info.width,
        )

        spacing_px = max(1, int(self.get_parameter("spacing_m").value / msg.info.resolution))
        min_len_px = max(2, int(self.get_parameter("min_segment_length_m").value / msg.info.resolution))

        cells = self.decompose_cells(grid, min_len_px)
        waypoints = self.generate_path(cells, spacing_px, grid)
        path = self.to_ros_path(waypoints, msg)

        self.latest_path = path
        self.path_pub.publish(path)

        self.get_logger().info(
            f"Custom planner: {len(cells)} cells, {len(path.poses)} poses"
        )

    def decompose_cells(self, grid, min_len_px):
        cells = {}
        previous = []
        next_cell_id = 0

        for y in range(grid.shape[0]):
            current = self.row_intervals(grid[y], min_len_px)
            if not current:
                previous = []
                continue

            active = []
            assigned = set()

            for curr_idx, curr in enumerate(current):
                parents = [
                    prev for prev in previous
                    if self.overlap((prev.x1, prev.x2), curr)
                ]

                if len(parents) == 1:
                    cell_id = parents[0].cell_id
                else:
                    cell_id = next_cell_id
                    next_cell_id += 1

                seg = Segment(y, curr[0], curr[1], cell_id)
                cells.setdefault(cell_id, Cell(cell_id, []))
                cells[cell_id].segments.append(seg)
                active.append(seg)
                assigned.add(curr_idx)

            previous = active

        return list(cells.values())

    def row_intervals(self, row, min_len_px):
        free_x = np.where(row == 0)[0]
        if len(free_x) == 0:
            return []

        runs = np.split(free_x, np.where(np.diff(free_x) > 1)[0] + 1)

        intervals = []
        for run in runs:
            if len(run) >= min_len_px:
                intervals.append((int(run[0]), int(run[-1])))

        return intervals

    def overlap(self, a, b):
        return a[0] <= b[1] and b[0] <= a[1]

    def generate_path(self, cells, spacing_px, grid):
        waypoints = []

        for cell in self.order_cells(cells):
            cell_waypoints = self.generate_cell_sweeps(cell, spacing_px, grid)

            if not cell_waypoints:
                continue

            if waypoints:
                transition = self.connect(waypoints[-1], cell_waypoints[0], grid)
                waypoints.extend(transition)

            waypoints.extend(cell_waypoints)

        return waypoints

    def order_cells(self, cells):
        return sorted(
            cells,
            key=lambda c: (
                min(s.y for s in c.segments),
                min(s.x1 for s in c.segments),
            ),
        )

    def generate_cell_sweeps(self, cell, spacing_px, grid):
        segments = sorted(cell.segments, key=lambda s: (s.y, s.x1))
        sampled = []
        last_y = None

        for seg in segments:
            if last_y is None or seg.y - last_y >= spacing_px:
                sampled.append(seg)
                last_y = seg.y

        if segments and sampled[-1].y != segments[-1].y:
            sampled.append(segments[-1])

        waypoints = []
        direction = 1

        for seg in sampled:
            if direction == 1:
                start = Waypoint(seg.x1, seg.y, 0.0)
                end = Waypoint(seg.x2, seg.y, 0.0)
            else:
                start = Waypoint(seg.x2, seg.y, math.pi)
                end = Waypoint(seg.x1, seg.y, math.pi)

            if waypoints:
                waypoints.extend(self.connect(waypoints[-1], start, grid))

            waypoints.extend(self.segment_waypoints(start, end, spacing_px))
            direction *= -1

        return waypoints

    def segment_waypoints(self, start, end, spacing_px):
        waypoints = []

        if start.x <= end.x:
            xs = range(start.x, end.x, spacing_px)
        else:
            xs = range(start.x, end.x, -spacing_px)

        for x in xs:
            waypoints.append(Waypoint(x, start.y, start.yaw))

        waypoints.append(end)
        return waypoints

    def connect(self, current, target, grid):
        waypoints = []

        vertical = Waypoint(
            current.x,
            target.y,
            math.pi / 2.0 if target.y > current.y else -math.pi / 2.0,
        )

        if current.y != target.y and self.line_safe(current, vertical, grid):
            waypoints.append(vertical)

        horizontal_start = waypoints[-1] if waypoints else current
        horizontal = Waypoint(
            target.x,
            target.y,
            0.0 if target.x > horizontal_start.x else math.pi,
        )

        if horizontal_start.x != target.x and self.line_safe(horizontal_start, horizontal, grid):
            waypoints.append(horizontal)

        return waypoints

    def line_safe(self, a, b, grid):
        step = int(self.get_parameter("validation_step_px").value)
        dist = max(abs(b.x - a.x), abs(b.y - a.y))
        steps = max(1, dist // max(1, step))

        for i in range(steps + 1):
            t = i / steps
            x = int(round(a.x + (b.x - a.x) * t))
            y = int(round(a.y + (b.y - a.y) * t))

            if y < 0 or y >= grid.shape[0] or x < 0 or x >= grid.shape[1]:
                return False

            if grid[y, x] != 0:
                return False

        return True

    def to_ros_path(self, waypoints, msg):
        path = Path()
        path.header.frame_id = msg.header.frame_id or "map"
        path.header.stamp.sec = 0
        path.header.stamp.nanosec = 0

        for wp in waypoints:
            x, y = self.pixel_to_map(wp.x, wp.y, msg)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(wp.yaw / 2.0)
            pose.pose.orientation.w = math.cos(wp.yaw / 2.0)

            path.poses.append(pose)

        return path

    def pixel_to_map(self, x_px, y_px, msg):
        resolution = msg.info.resolution
        origin = msg.info.origin

        local_x = (x_px + 0.5) * resolution
        local_y = (y_px + 0.5) * resolution
        yaw = self.quaternion_to_yaw(origin.orientation)

        x = origin.position.x + local_x * math.cos(yaw) - local_y * math.sin(yaw)
        y = origin.position.y + local_x * math.sin(yaw) + local_y * math.cos(yaw)

        return x, y

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_latest(self):
        if self.latest_path is not None:
            self.path_pub.publish(self.latest_path)


def main(args=None):
    rclpy.init(args=args)
    node = CustomPathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()