#!/usr/bin/env python3

import math
import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


class Segment:
    def __init__(self, y, x1, x2, cell_id=None):
        self.y = y
        self.x1 = x1
        self.x2 = x2
        self.cell_id = cell_id
        self.visited = False

    def is_connected(self, other, spacing_px):
        return (self.x1 - spacing_px) <= other.x2 and (self.x2 + spacing_px) >= other.x1


class Cell:
    def __init__(self, cell_id):
        self.id = cell_id
        self.segments = []

    def add_segment(self, segment):
        self.segments.append(segment)

    def first_segment(self):
        return min(self.segments, key=lambda s: (s.y, s.x1))


class PathGeneratorNode(Node):
    def __init__(self):
        super().__init__('path_generator_node')

        self.declare_parameter("map_topic", "/coverage/safe_map")
        self.declare_parameter("path_topic", "/coverage/path")
        self.declare_parameter("spacing_m", 0.3)
        self.declare_parameter("min_segment_length_m", 0.2)

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
        min_segment_length_m = self.get_parameter("min_segment_length_m").value

        grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height,
            msg.info.width
        )

        safe_free = (grid == 0).astype(np.uint8) * 255

        waypoints, cell_count = self.generate_boustrophedon_path(
            safe_free,
            msg.info.resolution,
            spacing_m,
            min_segment_length_m
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

        self.get_logger().info(
            f"Generated boustrophedon path with {len(path.poses)} waypoints "
            f"across {cell_count} cells."
        )

    def publish_latest(self):
        if self.latest_path is not None:
            self.path_pub.publish(self.latest_path)

    def generate_boustrophedon_path(self, safe_free, resolution, spacing_m, min_segment_length_m):
        spacing_px = max(1, int(spacing_m / resolution))
        min_segment_length_px = max(2, int(min_segment_length_m / resolution))
        cells = self.decompose_boustrophedon(safe_free, min_segment_length_px)
        cells = [cell for cell in cells if cell.segments]

        if not cells:
            return [], 0

        ordered_cells = self.order_cells(cells)
        waypoints = []

        for cell in ordered_cells:
            cell_waypoints = self.generate_cell_sweeps(cell, spacing_px)
            if not cell_waypoints:
                continue

            if waypoints:
                self.add_transition(waypoints, waypoints[-1], cell_waypoints[0])

            waypoints.extend(cell_waypoints)

        return waypoints, len(ordered_cells)

    def decompose_boustrophedon(self, safe_free, min_segment_length_px):
        cells = {}
        previous_intervals = []
        next_cell_id = 0

        for y in range(safe_free.shape[0]):
            current_intervals = self.row_intervals(safe_free[y, :], min_segment_length_px)

            if not current_intervals:
                previous_intervals = []
                continue

            components = self.interval_overlap_components(
                previous_intervals,
                current_intervals
            )
            current_active = []
            assigned_current = set()

            for prev_indices, curr_indices in components:
                if not curr_indices:
                    continue

                if len(prev_indices) == 1 and len(curr_indices) == 1:
                    cell_id = previous_intervals[prev_indices[0]][2]
                    curr_idx = curr_indices[0]
                    self.add_interval_to_cell(
                        cells, cell_id, y, current_intervals[curr_idx]
                    )
                    current_active.append((*current_intervals[curr_idx], cell_id))
                    assigned_current.add(curr_idx)
                    continue

                # Connectivity changed: split or merge. Start new cells after
                # the critical row, which is the core boustrophedon operation.
                for curr_idx in curr_indices:
                    cell_id = next_cell_id
                    next_cell_id += 1
                    self.add_interval_to_cell(
                        cells, cell_id, y, current_intervals[curr_idx]
                    )
                    current_active.append((*current_intervals[curr_idx], cell_id))
                    assigned_current.add(curr_idx)

            for curr_idx, interval in enumerate(current_intervals):
                if curr_idx in assigned_current:
                    continue
                cell_id = next_cell_id
                next_cell_id += 1
                self.add_interval_to_cell(cells, cell_id, y, interval)
                current_active.append((*interval, cell_id))

            previous_intervals = current_active

        return list(cells.values())

    def row_intervals(self, row, min_segment_length_px):
        xs = np.where(row > 0)[0]
        if len(xs) == 0:
            return []

        runs = np.split(xs, np.where(np.diff(xs) > 1)[0] + 1)
        intervals = []
        for run in runs:
            if len(run) >= min_segment_length_px:
                intervals.append((int(run[0]), int(run[-1])))
        return intervals

    def interval_overlap_components(self, previous_intervals, current_intervals):
        nodes = []
        edges = {}

        for i in range(len(previous_intervals)):
            node = ("p", i)
            nodes.append(node)
            edges[node] = []

        for i in range(len(current_intervals)):
            node = ("c", i)
            nodes.append(node)
            edges[node] = []

        for prev_idx, prev in enumerate(previous_intervals):
            for curr_idx, curr in enumerate(current_intervals):
                if self.intervals_overlap(prev[:2], curr):
                    p_node = ("p", prev_idx)
                    c_node = ("c", curr_idx)
                    edges[p_node].append(c_node)
                    edges[c_node].append(p_node)

        components = []
        visited = set()

        for node in nodes:
            if node in visited:
                continue

            stack = [node]
            visited.add(node)
            prev_indices = []
            curr_indices = []

            while stack:
                current = stack.pop()
                kind, idx = current
                if kind == "p":
                    prev_indices.append(idx)
                else:
                    curr_indices.append(idx)

                for neighbor in edges[current]:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        stack.append(neighbor)

            components.append((prev_indices, curr_indices))

        return components

    def intervals_overlap(self, a, b):
        return a[0] <= b[1] and b[0] <= a[1]

    def add_interval_to_cell(self, cells, cell_id, y, interval):
        if cell_id not in cells:
            cells[cell_id] = Cell(cell_id)

        x1, x2 = interval
        cells[cell_id].add_segment(Segment(y, x1, x2, cell_id))

    def order_cells(self, cells):
        ordered = []
        remaining = list(cells)
        curr_x = 0
        curr_y = 0

        while remaining:
            next_cell = min(
                remaining,
                key=lambda cell: self.distance_to_cell(cell, curr_x, curr_y)
            )
            ordered.append(next_cell)
            remaining.remove(next_cell)

            seg = next_cell.first_segment()
            curr_x = seg.x1
            curr_y = seg.y

        return ordered

    def distance_to_cell(self, cell, x, y):
        return min(
            min((seg.x1 - x) ** 2 + (seg.y - y) ** 2,
                (seg.x2 - x) ** 2 + (seg.y - y) ** 2)
            for seg in cell.segments
        )

    def generate_cell_sweeps(self, cell, spacing_px):
        sampled = self.sample_cell_segments(cell, spacing_px)
        if not sampled:
            return []

        waypoints = []
        direction = 1

        for seg in sampled:
            start = self.segment_start(seg, direction)
            if waypoints:
                self.add_transition(waypoints, waypoints[-1], start)

            self.add_segment_waypoints(waypoints, seg, direction, spacing_px)
            direction *= -1

        return waypoints

    def sample_cell_segments(self, cell, spacing_px):
        ordered = sorted(cell.segments, key=lambda seg: (seg.y, seg.x1))
        sampled = []
        last_y = None

        for seg in ordered:
            if last_y is None or seg.y - last_y >= spacing_px:
                sampled.append(seg)
                last_y = seg.y

        if ordered and sampled[-1].y != ordered[-1].y:
            sampled.append(ordered[-1])

        return sampled

    def segment_start(self, seg, direction):
        if direction == 1:
            return (seg.x1, seg.y, 0.0)
        return (seg.x2, seg.y, math.pi)

    def add_segment_waypoints(self, waypoints, seg, direction, spacing_px):
        if direction == 1:
            for x in range(seg.x1, seg.x2, spacing_px):
                waypoints.append((x, seg.y, 0.0))
            waypoints.append((seg.x2, seg.y, 0.0))
        else:
            for x in range(seg.x2, seg.x1, -spacing_px):
                waypoints.append((x, seg.y, math.pi))
            waypoints.append((seg.x1, seg.y, math.pi))

    def add_transition(self, waypoints, current, target):
        curr_x, curr_y, _ = current
        target_x, target_y, _ = target

        if curr_y != target_y:
            yaw = math.pi / 2.0 if target_y > curr_y else -math.pi / 2.0
            waypoints.append((curr_x, target_y, yaw))

        if curr_x != target_x:
            yaw = 0.0 if target_x > curr_x else math.pi
            waypoints.append((target_x, target_y, yaw))

    def pixel_to_map(self, x_px, y_px, msg):
        resolution = msg.info.resolution
        origin = msg.info.origin
        local_x = (x_px + 0.5) * resolution
        local_y = (y_px + 0.5) * resolution
        yaw = self.quaternion_to_yaw(origin.orientation)

        x = origin.position.x + local_x * math.cos(yaw) - local_y * math.sin(yaw)
        y = origin.position.y + local_x * math.sin(yaw) + local_y * math.cos(yaw)
        return x, y

    def quaternion_to_yaw(self, orientation):
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)
    
def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
