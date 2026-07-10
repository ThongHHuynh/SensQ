#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from coverage_planner.cell_decomposition import decompose_cells
from coverage_planner.grid_utils import (
    map_to_pixel,
    meters_to_pixels,
    occupancy_grid_to_array,
    pixel_to_map,
    transpose_grid,
)
from coverage_planner.path_simplifier import remove_duplicate_waypoints, simplify_path_rdp
from coverage_planner.path_validator import unsafe_edge_count
from coverage_planner.sweep_generator import generate_cell_sweeps, order_cells
from coverage_planner.transition_planner import connect


class ProductionPathGenerator(LifecycleNode):
    def __init__(self):
        super().__init__("production_path_gen")

        self.declare_parameter("map_topic", "/coverage/safe_map")
        self.declare_parameter("path_topic", "/coverage/path")
        self.declare_parameter("coverage_width_m", 0.3)
        self.declare_parameter("waypoint_spacing_m", 0.3)
        self.declare_parameter("min_segment_length_m", 0.2)
        self.declare_parameter("min_cell_area_m2", 0.2)
        self.declare_parameter("validation_step_m", 0.05)
        self.declare_parameter("transition_sample_m", 0.2)
        self.declare_parameter("reject_unsafe_path", True)
        self.declare_parameter("simplify_tolerance_m", 0.03)
        self.declare_parameter("optimize_cell_order", True)
        self.declare_parameter("sweep_direction", "auto")
        self.declare_parameter("use_diagonal_astar", True)
        self.declare_parameter("max_astar_iterations", 50_000)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("map_frame", "map")

        self.latest_map = None
        self.latest_path = None
        self.has_run = False

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.path_pub = self.create_publisher(
            Path, self.get_parameter("path_topic").value, qos
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.map_callback,
            qos,
        )

        # Service trigger: generate path on demand, not on every map message.
        self.generate_srv = self.create_service(
            Trigger, "~/generate_path", self.handle_generate
        )

        # TF for robot pose lookup.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.publish_latest)

        self.get_logger().info("Configured. Call ~/generate_path to trigger planning.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.destroy_publisher(self.path_pub)
        self.destroy_subscription(self.map_sub)
        self.destroy_timer(self.timer)
        self.latest_map = None
        self.latest_path = None
        self.has_run = False
        return TransitionCallbackReturn.SUCCESS

    def map_callback(self, msg):
        """Store map. Do NOT auto-generate path — wait for service call."""
        self.latest_map = msg
        self.get_logger().info(
            f"Map received: {msg.info.width}x{msg.info.height}, "
            f"resolution={msg.info.resolution:.3f}",
            once=True,
        )

    def handle_generate(self, request, response):
        """Service handler: generate coverage path on demand."""
        if self.latest_map is None:
            response.success = False
            response.message = "No map received yet."
            return response

        started = time.perf_counter()
        try:
            path = self.plan(self.latest_map)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            if path is None:
                response.success = False
                response.message = f"Path rejected after {elapsed_ms:.1f} ms."
            else:
                self.latest_path = path
                self.path_pub.publish(path)
                self.has_run = True
                response.success = True
                response.message = f"Published {len(path.poses)} poses in {elapsed_ms:.1f} ms."
                self.get_logger().info(
                    f"Planning completed in {elapsed_ms:.1f} ms."
                )
        except Exception as e:
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self.get_logger().error(f"Planning failed: {e}")
            response.success = False
            response.message = f"{e} ({elapsed_ms:.1f} ms)"

        return response

    def plan(self, msg):
        grid = occupancy_grid_to_array(msg)
        resolution = msg.info.resolution

        coverage_width_px = meters_to_pixels(
            self.get_parameter("coverage_width_m").value, resolution
        )
        waypoint_spacing_px = meters_to_pixels(
            self.get_parameter("waypoint_spacing_m").value, resolution
        )
        min_len_px = meters_to_pixels(
            self.get_parameter("min_segment_length_m").value, resolution, minimum=2
        )
        min_cell_area_px = int(
            self.get_parameter("min_cell_area_m2").value / (resolution * resolution)
        )
        validation_step_px = meters_to_pixels(
            self.get_parameter("validation_step_m").value, resolution
        )
        transition_sample_px = meters_to_pixels(
            self.get_parameter("transition_sample_m").value, resolution
        )
        simplify_tol_px = self.get_parameter("simplify_tolerance_m").value / resolution
        use_2opt = self.get_parameter("optimize_cell_order").value
        use_diagonal = self.get_parameter("use_diagonal_astar").value
        max_astar_iter = self.get_parameter("max_astar_iterations").value

        # Choose sweep direction: horizontal, vertical, or auto.
        sweep_direction = self.get_parameter("sweep_direction").value
        transposed = False

        if sweep_direction == "vertical":
            grid = transpose_grid(grid)
            transposed = True
        elif sweep_direction == "auto":
            grid, transposed = self.pick_best_direction(
                grid, min_len_px, min_cell_area_px, coverage_width_px
            )

        # Get robot start position from TF.
        start_x, start_y = self.get_robot_pixel_position(msg)

        # If grid was transposed, swap start coordinates to match.
        if transposed:
            start_x, start_y = start_y, start_x

        cells = decompose_cells(grid, min_len_px, min_cell_area_px)
        self.get_logger().info(f"Decomposed into {len(cells)} cells.")

        waypoints = self.generate_full_path(
            cells, grid, coverage_width_px, waypoint_spacing_px,
            validation_step_px, transition_sample_px, use_2opt, use_diagonal,
            start_x, start_y, max_astar_iter,
        )

        # If grid was transposed, swap waypoint coordinates back.
        if transposed:
            for wp in waypoints:
                wp.x, wp.y = wp.y, wp.x

        if not waypoints:
            self.get_logger().error("No coverage waypoints generated.")
            return None

        validation_grid = occupancy_grid_to_array(msg) if transposed else grid
        waypoints = remove_duplicate_waypoints(waypoints)

        if simplify_tol_px > 0:
            before = len(waypoints)
            simplified = simplify_path_rdp(waypoints, simplify_tol_px)
            simplified_unsafe = unsafe_edge_count(
                simplified, validation_grid, validation_step_px
            )
            if simplified_unsafe:
                self.get_logger().warn(
                    "Path simplification would create "
                    f"{simplified_unsafe} unsafe edges; keeping raw path."
                )
            else:
                waypoints = simplified
                self.get_logger().info(
                    f"Simplified {before} -> {len(waypoints)} waypoints."
                )

        unsafe_edges = unsafe_edge_count(waypoints, validation_grid, validation_step_px)
        if unsafe_edges:
            self.get_logger().warn(f"Path has {unsafe_edges} unsafe edges.")
            if self.get_parameter("reject_unsafe_path").value:
                self.get_logger().error("Rejected unsafe path.")
                return None

        path = self.to_ros_path(waypoints, msg)
        self.get_logger().info(f"Published {len(path.poses)} poses across {len(cells)} cells.")
        return path

    def pick_best_direction(self, grid, min_len_px, min_cell_area_px, spacing_px):
        """Try horizontal and vertical, pick whichever produces fewer cells.

        Fewer cells means fewer inter-cell transitions, which typically
        results in a shorter total path.
        """
        h_cells = decompose_cells(grid, min_len_px, min_cell_area_px)
        v_grid = transpose_grid(grid)
        v_cells = decompose_cells(v_grid, min_len_px, min_cell_area_px)

        if len(v_cells) < len(h_cells):
            self.get_logger().info(
                f"Auto sweep: vertical ({len(v_cells)} cells vs {len(h_cells)} horizontal)."
            )
            return v_grid, True

        self.get_logger().info(
            f"Auto sweep: horizontal ({len(h_cells)} cells vs {len(v_cells)} vertical)."
        )
        return grid, False

    def get_robot_pixel_position(self, msg):
        """Look up robot position from TF and convert to pixel coordinates.

        Falls back to (0, 0) if TF is not available.
        """
        map_frame = self.get_parameter("map_frame").value
        base_frame = self.get_parameter("robot_base_frame").value

        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            px, py = map_to_pixel(robot_x, robot_y, msg)
            self.get_logger().info(f"Robot at map ({robot_x:.2f}, {robot_y:.2f}) → pixel ({px}, {py}).")
            return px, py
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed ({e}), using (0, 0) as start.")
            return 0, 0

    def generate_full_path(self, cells, grid, coverage_width_px, waypoint_spacing_px,
                           validation_step_px, transition_sample_px, use_2opt,
                           use_diagonal, start_x, start_y, max_astar_iter):
        path = []
        skipped_cells = 0
        for cell in order_cells(cells, start_x, start_y, use_2opt):
            cell_path = self.generate_cell_path(
                cell, grid, coverage_width_px, waypoint_spacing_px,
                validation_step_px, transition_sample_px, use_diagonal,
                max_astar_iter,
            )
            if not cell_path:
                skipped_cells += 1
                continue

            if path and not self.same_point(path[-1], cell_path[0]):
                transition = connect(
                    path[-1], cell_path[0], grid,
                    validation_step_px, transition_sample_px, use_diagonal,
                    max_astar_iterations=max_astar_iter,
                )
                if not transition:
                    skipped_cells += 1
                    self.get_logger().warn(
                        f"Skipping unreachable cell {cell.id}; no safe transition found."
                    )
                    continue
                path.extend(transition)

            path.extend(cell_path)

        if skipped_cells:
            self.get_logger().warn(
                f"Skipped {skipped_cells} unreachable/invalid cells."
            )

        return path

    def generate_cell_path(self, cell, grid, coverage_width_px, waypoint_spacing_px,
                           validation_step_px, transition_sample_px, use_diagonal,
                           max_astar_iter):
        path = []
        tagged_sweeps = generate_cell_sweeps(
            cell, coverage_width_px, waypoint_spacing_px
        )

        for tag, waypoint in tagged_sweeps:
            if tag == "transition" and path:
                transition = connect(
                    path[-1], waypoint, grid,
                    validation_step_px, transition_sample_px, use_diagonal,
                    max_astar_iterations=max_astar_iter,
                )
                if not transition and not self.same_point(path[-1], waypoint):
                    self.get_logger().warn(
                        f"Skipping cell {cell.id}; no safe inter-row transition found."
                    )
                    return []
                path.extend(transition)
                continue

            path.append(waypoint)

        return path

    def same_point(self, a, b):
        return int(round(a.x)) == int(round(b.x)) and int(round(a.y)) == int(round(b.y))

    def to_ros_path(self, waypoints, msg):
        path = Path()
        path.header.frame_id = msg.header.frame_id or "map"
        path.header.stamp.sec = 0
        path.header.stamp.nanosec = 0

        for waypoint in waypoints:
            x, y = pixel_to_map(waypoint.x, waypoint.y, msg)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
            pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
            path.poses.append(pose)

        return path

    def publish_latest(self):
        if self.latest_path is not None:
            self.path_pub.publish(self.latest_path)


def main(args=None):
    rclpy.init(args=args)
    node = ProductionPathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
