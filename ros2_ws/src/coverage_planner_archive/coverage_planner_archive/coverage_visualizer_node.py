#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from coverage_planner_archive.cell_decomposition import decompose_cells
from coverage_planner_archive.grid_utils import (
    meters_to_pixels,
    occupancy_grid_to_array,
    pixel_to_map,
    transpose_grid,
)


class CoverageVisualizerNode(Node):
    def __init__(self):
        super().__init__("coverage_visualizer_node")

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE

        self.declare_parameter("min_segment_length_m", 0.2)
        self.declare_parameter("min_cell_area_m2", 0.2)
        self.declare_parameter("sweep_direction", "auto")
        self.declare_parameter("map_topic", "/coverage/safe_map")
        self.declare_parameter("show_cell_labels", True)
        self.declare_parameter("cell_label_height_m", 0.25)
        self.declare_parameter("cell_label_z_m", 0.2)

        self.path_sub = self.create_subscription(
            Path,
            "/coverage/path",
            self.path_callback,
            qos_profile=qos_profile
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.map_callback,
            qos_profile=qos_profile
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/coverage/waypoint_markers",
            qos_profile=qos_profile
        )

        self.cell_marker_pub = self.create_publisher(
            MarkerArray,
            "/coverage/cell_markers",
            qos_profile=qos_profile
        )
        
        self.latest_markers = None
        self.latest_cell_markers = None
        self.timer = self.create_timer(1.0, self.publish_latest)
        
        self.get_logger().info("Coverage Visualizer Node has been started.")

    def path_callback(self, msg):
        markers = MarkerArray()

        # Delete all old markers first
        delete_marker = Marker()
        delete_marker.header = msg.header
        delete_marker.header.stamp.sec = 0
        delete_marker.header.stamp.nanosec = 0
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        # Sphere list for all waypoints
        marker = Marker()
        marker.header = msg.header
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
        marker.color.g = 0.3
        marker.color.b = 0.0
        marker.color.a = 1.0

        for pose in msg.poses:
            marker.points.append(pose.pose.position)

        markers.markers.append(marker)

        # Numbered text labels at every Nth waypoint
        total = len(msg.poses)
        # Adapt label spacing: show ~50 labels max to keep it readable
        label_step = max(1, total // 50)
        for i, pose in enumerate(msg.poses):
            if i % label_step != 0 and i != total - 1:
                continue
            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.header.stamp.sec = 0
            text_marker.header.stamp.nanosec = 0
            text_marker.ns = "coverage_labels"
            text_marker.id = i + 1  # offset by 1 to avoid collision with sphere id=0
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = pose.pose
            text_marker.pose.position.z = 0.15  # float above the sphere
            text_marker.scale.z = 0.12  # text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.9
            text_marker.text = str(i)
            markers.markers.append(text_marker)

        self.latest_markers = markers
        self.marker_pub.publish(self.latest_markers)

    def map_callback(self, msg):
        grid = occupancy_grid_to_array(msg)
        resolution = msg.info.resolution
        
        min_len_px = meters_to_pixels(
            self.get_parameter("min_segment_length_m").value, resolution, minimum=2
        )
        min_cell_area_px = int(
            self.get_parameter("min_cell_area_m2").value / (resolution * resolution)
        )
        sweep_direction = self.get_parameter("sweep_direction").value
        transposed = False
        
        if sweep_direction == "vertical":
            grid = transpose_grid(grid)
            transposed = True
        elif sweep_direction == "auto":
            h_cells = decompose_cells(grid, min_len_px, min_cell_area_px)
            v_grid = transpose_grid(grid)
            v_cells = decompose_cells(v_grid, min_len_px, min_cell_area_px)
            
            if len(v_cells) < len(h_cells):
                grid = v_grid
                transposed = True

        cells = decompose_cells(grid, min_len_px, min_cell_area_px)
        
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header = msg.header
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        show_labels = self.get_parameter("show_cell_labels").value
        label_height = float(self.get_parameter("cell_label_height_m").value)
        label_z = float(self.get_parameter("cell_label_z_m").value)

        for cell in cells:
            if not cell.segments:
                continue
            
            min_x, min_y, max_x, max_y = self.cell_bounds(cell, transposed)

            p1_x, p1_y = pixel_to_map(min_x, min_y, msg)
            p2_x, p2_y = pixel_to_map(max_x, min_y, msg)
            p3_x, p3_y = pixel_to_map(max_x, max_y, msg)
            p4_x, p4_y = pixel_to_map(min_x, max_y, msg)

            marker = Marker()
            marker.header = msg.header
            marker.header.stamp.sec = 0
            marker.header.stamp.nanosec = 0
            marker.ns = "cells"
            marker.id = cell.id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.points = [
                Point(x=p1_x, y=p1_y, z=0.0),
                Point(x=p2_x, y=p2_y, z=0.0),
                Point(x=p3_x, y=p3_y, z=0.0),
                Point(x=p4_x, y=p4_y, z=0.0),
                Point(x=p1_x, y=p1_y, z=0.0),
            ]
            marker_array.markers.append(marker)

            if show_labels:
                label_x_px, label_y_px = self.cell_centroid(cell, transposed)
                label_x, label_y = pixel_to_map(label_x_px, label_y_px, msg)
                label = Marker()
                label.header = msg.header
                label.header.stamp.sec = 0
                label.header.stamp.nanosec = 0
                label.ns = "cell_labels"
                label.id = cell.id
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = label_x
                label.pose.position.y = label_y
                label.pose.position.z = label_z
                label.pose.orientation.w = 1.0
                label.scale.z = label_height
                label.color.r = 0.1
                label.color.g = 1.0
                label.color.b = 0.2
                label.color.a = 1.0
                label.text = str(cell.id)
                marker_array.markers.append(label)
            
        self.latest_cell_markers = marker_array
        self.cell_marker_pub.publish(self.latest_cell_markers)

    def cell_bounds(self, cell, transposed):
        min_y = min(s.y for s in cell.segments)
        max_y = max(s.y for s in cell.segments)
        min_x = min(s.x1 for s in cell.segments)
        max_x = max(s.x2 for s in cell.segments)

        if transposed:
            return min_y, min_x, max_y, max_x
        return min_x, min_y, max_x, max_y

    def cell_centroid(self, cell, transposed):
        x_px, y_px = cell.centroid
        if transposed:
            return y_px, x_px
        return x_px, y_px

    def publish_latest(self):
        if self.latest_markers is not None:
            self.marker_pub.publish(self.latest_markers)
        if self.latest_cell_markers is not None:
            self.cell_marker_pub.publish(self.latest_cell_markers)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
