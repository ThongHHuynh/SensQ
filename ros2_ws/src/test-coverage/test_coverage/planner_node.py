#!/usr/bin/env python3

from __future__ import annotations

import colorsys
import json
import math
import threading

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, Int32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from test_coverage.grid_map import GridMap, sweep_to_grid
from test_coverage.models import PathKind, PlannerConfig, Point2D
from test_coverage.planner import BoustrophedonCoveragePlanner
from test_coverage.segmentation import coverage_segment_starts


class CoveragePlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("coverage_planner")
        self._declare_parameters()
        self._map_message = None
        self._planning_lock = threading.Lock()
        self._auto_generated = False

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._map_callback,
            latched_qos,
        )
        self._path_publisher = self.create_publisher(
            Path, self.get_parameter("path_topic").value, latched_qos
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, latched_qos
        )
        self._segment_publisher = self.create_publisher(
            Int32MultiArray,
            self.get_parameter("segment_topic").value,
            latched_qos,
        )
        self._status_publisher = self.create_publisher(
            String, self.get_parameter("status_topic").value, latched_qos
        )
        self._generate_service = self.create_service(
            Trigger, "~/generate", self._generate_callback
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._auto_timer = self.create_timer(1.0, self._maybe_auto_generate)
        self.get_logger().info("Ready; call ~/generate after Nav2 has a map and TF.")

    def _declare_parameters(self) -> None:
        declarations = {
            "map_topic": "/map",
            "path_topic": "/test_coverage/path",
            "marker_topic": "/test_coverage/markers",
            "segment_topic": "/test_coverage/segment_starts",
            "status_topic": "/test_coverage/planning_status",
            "map_frame": "map",
            "robot_base_frame": "base_footprint",
            "operation_width_m": 0.25,
            "overlap_m": 0.03,
            "robot_radius_m": 0.10,
            "safety_margin_m": 0.05,
            "headland_depth_m": 0.20,
            "headland_overlap_m": 0.05,
            "min_turning_radius_m": 0.12,
            "path_spacing_m": 0.05,
            "min_lane_length_m": 0.20,
            "min_cell_area_m2": 0.05,
            "occupied_threshold": 65,
            "unknown_is_obstacle": True,
            "sweep_mode": "auto",
            "turn_mode": "auto",
            "max_astar_iterations": 100000,
            "optimize_cell_order": True,
            "auto_generate": False,
        }
        for name, default in declarations.items():
            self.declare_parameter(name, default)

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._map_message = message
        self.get_logger().info(
            f"Map received: {message.info.width}x{message.info.height} at "
            f"{message.info.resolution:.3f} m/cell",
            once=True,
        )

    def _generate_callback(self, _request, response):
        success, message = self._generate()
        response.success = success
        response.message = message
        return response

    def _maybe_auto_generate(self) -> None:
        if (
            self._auto_generated
            or not self.get_parameter("auto_generate").value
            or self._map_message is None
        ):
            return
        success, message = self._generate()
        if success:
            self._auto_generated = True
        else:
            self.get_logger().warn(message, throttle_duration_sec=5.0)

    def _generate(self) -> tuple[bool, str]:
        if self._map_message is None:
            return False, "No OccupancyGrid received"
        if not self._planning_lock.acquire(blocking=False):
            return False, "Planning is already in progress"

        try:
            map_frame = (
                self._map_message.header.frame_id
                or self.get_parameter("map_frame").value
            )
            base_frame = self.get_parameter("robot_base_frame").value
            try:
                transform = self._tf_buffer.lookup_transform(
                    map_frame,
                    base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0),
                )
            except TransformException as error:
                return False, f"Robot pose unavailable: {error}"

            grid_map = GridMap.from_message(self._map_message)
            start = Point2D(
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
            )
            planner = BoustrophedonCoveragePlanner(self._planner_config())
            plan = planner.plan(grid_map, start)
            path = self._to_path(plan, grid_map)
            markers = self._to_markers(
                plan,
                grid_map,
                self.get_clock().now().to_msg(),
            )
            self._publish_segments(plan.points)
            self._path_publisher.publish(path)
            self._marker_publisher.publish(markers)

            metrics = {
                "state": "READY",
                "poses": len(path.poses),
                "axis": plan.metrics.axis.value,
                "cells": plan.metrics.cell_count,
                "path_length_m": round(plan.metrics.path_length_m, 3),
                "entry_distance_m": round(plan.metrics.entry_distance_m, 3),
                "estimated_coverage_percent": round(
                    plan.metrics.estimated_coverage_percent, 2
                ),
                "curve_turns": plan.metrics.curve_turns,
                "square_turns": plan.metrics.square_turns,
                "transits": plan.metrics.transit_segments,
                "planning_time_ms": round(plan.metrics.planning_time_ms, 1),
                "warnings": plan.warnings,
            }
            self._publish_status(metrics)
            message = json.dumps(metrics, separators=(",", ":"))
            self.get_logger().info(message)
            return True, message
        except Exception as error:  # service boundary must return a useful failure
            self.get_logger().error(f"Coverage planning failed: {error}")
            self._publish_status({"state": "ERROR", "message": str(error)})
            return False, str(error)
        finally:
            self._planning_lock.release()

    def _planner_config(self) -> PlannerConfig:
        def value(name):
            return self.get_parameter(name).value

        return PlannerConfig(
            operation_width_m=float(value("operation_width_m")),
            overlap_m=float(value("overlap_m")),
            robot_radius_m=float(value("robot_radius_m")),
            safety_margin_m=float(value("safety_margin_m")),
            headland_depth_m=float(value("headland_depth_m")),
            headland_overlap_m=float(value("headland_overlap_m")),
            min_turning_radius_m=float(value("min_turning_radius_m")),
            path_spacing_m=float(value("path_spacing_m")),
            min_lane_length_m=float(value("min_lane_length_m")),
            min_cell_area_m2=float(value("min_cell_area_m2")),
            occupied_threshold=int(value("occupied_threshold")),
            unknown_is_obstacle=bool(value("unknown_is_obstacle")),
            sweep_mode=str(value("sweep_mode")),
            turn_mode=str(value("turn_mode")),
            max_astar_iterations=int(value("max_astar_iterations")),
            optimize_cell_order=bool(value("optimize_cell_order")),
        )

    def _to_path(self, plan, grid_map: GridMap) -> Path:
        path = Path()
        path.header.frame_id = plan.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for point in plan.points:
            world = grid_map.grid_to_world(point.point)
            pose = self._new_pose(path.header, world, point.yaw + grid_map.origin_yaw)
            path.poses.append(pose)
        return path

    @staticmethod
    def _new_pose(header, point: Point2D, yaw: float):
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    @staticmethod
    def _to_markers(plan, grid_map: GridMap, stamp) -> MarkerArray:
        path_colors = {
            PathKind.COVERAGE: ColorRGBA(r=0.10, g=0.75, b=0.20, a=1.0),
            PathKind.CURVE_TURN: ColorRGBA(r=0.15, g=0.45, b=1.0, a=1.0),
            PathKind.SQUARE_TURN: ColorRGBA(r=1.0, g=0.65, b=0.05, a=1.0),
            PathKind.TRANSIT: ColorRGBA(r=0.80, g=0.15, b=0.85, a=1.0),
        }
        marker_array = MarkerArray()

        clear = Marker()
        clear.header.frame_id = plan.frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for cell in plan.cells:
            fill = Marker()
            fill.header.frame_id = plan.frame_id
            fill.header.stamp = stamp
            fill.ns = "coverage_cells"
            fill.id = cell.cell_id
            fill.type = Marker.CUBE_LIST
            fill.action = Marker.ADD
            fill.pose.position.x = grid_map.origin_x
            fill.pose.position.y = grid_map.origin_y
            fill.pose.orientation.z = math.sin(0.5 * grid_map.origin_yaw)
            fill.pose.orientation.w = math.cos(0.5 * grid_map.origin_yaw)
            fill.scale.x = grid_map.resolution
            fill.scale.y = grid_map.resolution
            fill.scale.z = 0.015
            fill.color = CoveragePlannerNode._cell_color(
                cell.cell_id,
                alpha=0.24,
            )
            for segment in cell.segments:
                for along in range(segment.start, segment.end + 1):
                    grid_point = sweep_to_grid(
                        float(along),
                        float(segment.row),
                        cell.axis,
                    )
                    fill.points.append(
                        Point(
                            x=(grid_point.x + 0.5) * grid_map.resolution,
                            y=(grid_point.y + 0.5) * grid_map.resolution,
                            z=0.01,
                        )
                    )
            marker_array.markers.append(fill)

            label_segment = cell.segments[len(cell.segments) // 2]
            label_grid = sweep_to_grid(
                0.5 * (label_segment.start + label_segment.end),
                float(label_segment.row),
                cell.axis,
            )
            label_position = grid_map.grid_to_world(label_grid)
            label = Marker()
            label.header.frame_id = plan.frame_id
            label.header.stamp = stamp
            label.ns = "coverage_cell_labels"
            label.id = cell.cell_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = label_position.x
            label.pose.position.y = label_position.y
            label.pose.position.z = 0.14
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f"Cell {cell.cell_id}"
            marker_array.markers.append(label)

        path_marker_id = 0
        current_kind = None
        current_marker = None

        for path_point in plan.points:
            if path_point.kind != current_kind:
                current_kind = path_point.kind
                current_marker = Marker()
                current_marker.header.frame_id = plan.frame_id
                current_marker.header.stamp = stamp
                current_marker.ns = "coverage_path"
                current_marker.id = path_marker_id
                path_marker_id += 1
                current_marker.type = Marker.LINE_STRIP
                current_marker.action = Marker.ADD
                current_marker.pose.orientation.w = 1.0
                current_marker.scale.x = 0.04
                current_marker.color = path_colors[current_kind]
                marker_array.markers.append(current_marker)
            world = grid_map.grid_to_world(path_point.point)
            current_marker.points.append(Point(x=world.x, y=world.y, z=0.06))

        start = Marker()
        start.header.frame_id = plan.frame_id
        start.header.stamp = stamp
        start.ns = "coverage_start"
        start.id = 0
        start.type = Marker.SPHERE
        start.action = Marker.ADD
        first = grid_map.grid_to_world(plan.points[0].point)
        start.pose.position.x = first.x
        start.pose.position.y = first.y
        start.pose.position.z = 0.10
        start.pose.orientation.w = 1.0
        start.scale.x = start.scale.y = start.scale.z = 0.15
        start.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker_array.markers.append(start)

        end = Marker()
        end.header.frame_id = plan.frame_id
        end.header.stamp = stamp
        end.ns = "coverage_end"
        end.id = 0
        end.type = Marker.SPHERE
        end.action = Marker.ADD
        last = grid_map.grid_to_world(plan.points[-1].point)
        end.pose.position.x = last.x
        end.pose.position.y = last.y
        end.pose.position.z = 0.10
        end.pose.orientation.w = 1.0
        end.scale.x = end.scale.y = end.scale.z = 0.15
        end.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        marker_array.markers.append(end)
        return marker_array

    @staticmethod
    def _cell_color(cell_id: int, alpha: float) -> ColorRGBA:
        hue = (0.11 + cell_id * 0.61803398875) % 1.0
        red, green, blue = colorsys.hsv_to_rgb(hue, 0.72, 0.95)
        return ColorRGBA(r=red, g=green, b=blue, a=alpha)

    def _publish_status(self, payload: dict) -> None:
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._status_publisher.publish(message)

    def _publish_segments(self, points) -> None:
        message = Int32MultiArray()
        message.data = [len(points), *coverage_segment_starts(points)]
        self._segment_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoveragePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
