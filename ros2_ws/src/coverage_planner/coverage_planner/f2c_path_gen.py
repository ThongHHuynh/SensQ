#!/usr/bin/env python3

import heapq
import math
import cv2
import numpy as np
import rclpy
import fields2cover as f2c

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class Fields2CoverPathGeneratorNode(Node):
    def __init__(self):
        super().__init__("f2c_path_generator_node")

        self.declare_parameter("input_mode", "wkt")
        self.declare_parameter("map_topic", "/coverage/safe_map")
        self.declare_parameter("wkt_topic", "/coverage/opennav_wkt")
        self.declare_parameter("path_topic", "/coverage/path")
        self.declare_parameter("swath_marker_topic", "/coverage/f2c_swaths")
        self.declare_parameter("robot_width_m", 0.35)
        self.declare_parameter("coverage_width_m", 0.30)
        self.declare_parameter("min_turning_radius_m", 0.50)
        self.declare_parameter("sweep_angle_rad", 0.0)
        self.declare_parameter("use_best_swath_angle", True)
        self.declare_parameter("swath_angle_step_rad", 0.10)
        self.declare_parameter("allow_overlap", False)
        self.declare_parameter("swath_objective", "field_coverage")
        self.declare_parameter("path_output_mode", "swaths")
        self.declare_parameter("path_step_m", 0.10)
        self.declare_parameter("contour_simplification_px", 2.0)
        self.declare_parameter("min_cell_area_m2", 0.10)
        self.declare_parameter("min_hole_area_m2", 0.02)
        self.declare_parameter("path_validation_step_m", 0.05)
        self.declare_parameter("repair_unsafe_connections", True)
        self.declare_parameter("allow_diagonal_repair", False)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.path_pub = self.create_publisher(
            Path,
            self.get_parameter("path_topic").value,
            qos,
        )
        self.swath_marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("swath_marker_topic").value,
            qos,
        )

        self.input_mode = str(self.get_parameter("input_mode").value)
        self.latest_map_msg = None

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.map_callback,
            qos,
        )
        self.wkt_sub = None
        if self.input_mode == "wkt":
            self.wkt_sub = self.create_subscription(
                String,
                self.get_parameter("wkt_topic").value,
                self.wkt_callback,
                qos,
            )

        self.latest_path = None
        self.latest_swath_markers = None
        self.has_run = False
        self.timer = self.create_timer(1.0, self.publish_latest)

    def map_callback(self, msg):
        self.latest_map_msg = msg
        if self.input_mode == "wkt":
            return

        if self.has_run:
            return

        self.has_run = True
        cells = self.occupancy_grid_to_cells(msg)

        if cells is None or cells.size() == 0:
            self.get_logger().warn("No valid free-space polygon found.")
            return

        ros_path = self.plan_cells_to_ros_path(cells, msg)

        self.latest_path = ros_path
        self.path_pub.publish(ros_path)

        self.get_logger().info(
            f"Published Fields2Cover path with {len(ros_path.poses)} poses."
        )

    def wkt_callback(self, msg):
        if self.has_run:
            return

        if self.latest_map_msg is None:
            self.get_logger().warn("WKT received, waiting for /coverage/safe_map for validation.")
            return

        self.has_run = True
        cells = self.wkt_to_cells(msg.data)

        if cells is None or cells.size() == 0:
            self.get_logger().warn("No valid WKT polygon received for Fields2Cover.")
            return

        ros_path = self.plan_cells_to_ros_path(cells, self.latest_map_msg)

        self.latest_path = ros_path
        self.path_pub.publish(ros_path)

        self.get_logger().info(
            f"Published Fields2Cover path from WKT with {len(ros_path.poses)} poses."
        )

    def wkt_to_cells(self, wkt):
        cells = f2c.Cells()
        for polygon_wkt in self.split_wkt_polygons(wkt):
            cell = f2c.Cell()
            cell.importFromWkt(polygon_wkt)
            if not cell.isEmpty():
                cells.addGeometry(cell)

        if cells.size() == 0:
            return None

        self.get_logger().info(
            f"Converted OpenNav WKT to {cells.size()} Fields2Cover cell(s)."
        )
        return cells

    def split_wkt_polygons(self, wkt):
        text = wkt.strip()
        if text.upper().startswith("POLYGON"):
            return [text]
        if not text.upper().startswith("MULTIPOLYGON"):
            return []

        body_start = text.find("(")
        body_end = text.rfind(")")
        if body_start < 0 or body_end <= body_start:
            return []

        body = text[body_start + 1:body_end]
        polygons = []
        depth = 0
        start = None
        for idx, char in enumerate(body):
            if char == "(":
                if depth == 0:
                    start = idx
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0 and start is not None:
                    polygons.append(f"POLYGON {body[start:idx + 1]}")
                    start = None

        return polygons

    def occupancy_grid_to_cells(self, msg):
        min_cell_area_m2 = float(self.get_parameter("min_cell_area_m2").value)
        min_hole_area_m2 = float(self.get_parameter("min_hole_area_m2").value)
        simplify_px = float(self.get_parameter("contour_simplification_px").value)
        min_cell_area_px = min_cell_area_m2 / (msg.info.resolution ** 2)
        min_hole_area_px = min_hole_area_m2 / (msg.info.resolution ** 2)

        grid = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height,
            msg.info.width,
        )

        free = (grid == 0).astype(np.uint8) * 255

        contours, hierarchy = cv2.findContours(
            free,
            cv2.RETR_CCOMP,
            cv2.CHAIN_APPROX_NONE,
        )

        if not contours or hierarchy is None:
            return None

        hierarchy = hierarchy[0]
        cells = f2c.Cells()

        for contour_idx, contour in enumerate(contours):
            parent_idx = hierarchy[contour_idx][3]
            if parent_idx != -1:
                continue

            if cv2.contourArea(contour) < min_cell_area_px:
                continue

            exterior = self.contour_to_wkt_ring(contour, msg, simplify_px)
            if exterior is None:
                continue

            rings = [exterior]
            hole_idx = hierarchy[contour_idx][2]
            while hole_idx != -1:
                hole = contours[hole_idx]
                if cv2.contourArea(hole) >= min_hole_area_px:
                    ring = self.contour_to_wkt_ring(hole, msg, simplify_px)
                    if ring is not None:
                        rings.append(ring)
                hole_idx = hierarchy[hole_idx][0]

            cell = f2c.Cell()
            cell.importFromWkt(f"POLYGON ({','.join(rings)})")
            if not cell.isEmpty() and cell.area() >= min_cell_area_m2:
                cells.addGeometry(cell)

        self.get_logger().info(
            f"Converted safe map to {cells.size()} Fields2Cover cells."
        )
        return cells

    def contour_to_wkt_ring(self, contour, msg, simplify_px):
        contour = cv2.approxPolyDP(contour, simplify_px, True)
        points = []

        for p in contour[:, 0, :]:
            x, y = self.pixel_to_map(int(p[0]), int(p[1]), msg)
            if points and points[-1] == (x, y):
                continue
            points.append((x, y))

        if len(points) < 3:
            return None

        if points[0] != points[-1]:
            points.append(points[0])

        coords = ",".join(f"{x:.6f} {y:.6f}" for x, y in points)
        return f"({coords})"

    def plan_cells_to_ros_path(self, cells, map_msg):
        ros_path = Path()
        ros_path.header.frame_id = map_msg.header.frame_id or "map"
        ros_path.header.stamp.sec = 0
        ros_path.header.stamp.nanosec = 0
        swath_markers = self.create_marker_reset(ros_path.header)
        marker_id = 1

        for i in range(cells.size()):
            cell = cells.getGeometry(i)
            try:
                if str(self.get_parameter("path_output_mode").value) == "swaths":
                    swaths = self.generate_sorted_swaths(cell)
                    marker_id = self.append_swath_markers(
                        swaths,
                        swath_markers,
                        ros_path.header,
                        marker_id,
                    )
                    self.append_swaths_to_ros_path(swaths, ros_path, map_msg)
                    continue

                f2c_path = self.plan_fields2cover_path(cell)
            except Exception as exc:
                self.get_logger().warn(f"Skipping cell {i}: {exc}")
                continue

            self.append_f2c_path_to_ros_path(f2c_path, ros_path, map_msg)

        self.latest_swath_markers = swath_markers
        self.swath_marker_pub.publish(swath_markers)
        return ros_path

    def generate_sorted_swaths(self, cell):
        robot_width = float(self.get_parameter("robot_width_m").value)
        coverage_width = float(self.get_parameter("coverage_width_m").value)
        sweep_angle = float(self.get_parameter("sweep_angle_rad").value)
        use_best_angle = bool(self.get_parameter("use_best_swath_angle").value)
        angle_step = float(self.get_parameter("swath_angle_step_rad").value)
        allow_overlap = bool(self.get_parameter("allow_overlap").value)

        robot = f2c.Robot(robot_width, coverage_width)

        swath_gen = f2c.SG_BruteForce()
        swath_gen.setAllowOverlap(allow_overlap)
        swath_gen.setStepAngle(angle_step)
        if use_best_angle:
            objective = self.create_swath_objective()
            swaths = swath_gen.generateBestSwaths(
                objective,
                robot.getCovWidth(),
                cell,
            )
        else:
            swaths = swath_gen.generateSwaths(
                sweep_angle,
                robot.getCovWidth(),
                cell,
            )

        if swaths.size() == 0:
            return f2c.Swaths()

        sorter = f2c.RP_Snake()
        return sorter.genSortedSwaths(swaths)

    def plan_fields2cover_path(self, cell):
        min_turn_radius = float(self.get_parameter("min_turning_radius_m").value)
        path_step = float(self.get_parameter("path_step_m").value)

        robot_width = float(self.get_parameter("robot_width_m").value)
        coverage_width = float(self.get_parameter("coverage_width_m").value)
        robot = f2c.Robot(robot_width, coverage_width)
        robot.setMinTurningRadius(min_turn_radius)

        swaths = self.generate_sorted_swaths(cell)

        planner = f2c.PP_PathPlanning()
        dubins = f2c.PP_DubinsCurves()

        path = planner.planPath(robot, swaths, dubins)
        path.discretize(path_step)

        return path

    def append_swaths_to_ros_path(self, swaths, ros_path, map_msg):
        safe_grid = np.array(map_msg.data, dtype=np.int16).reshape(
            map_msg.info.height,
            map_msg.info.width,
        )
        path_step = float(self.get_parameter("path_step_m").value)
        rejected = 0
        disconnected_starts = 0

        for i in range(swaths.size()):
            swath = swaths.at(i)
            start = swath.startPoint()
            end = swath.endPoint()
            x1 = start.getX()
            y1 = start.getY()
            x2 = end.getX()
            y2 = end.getY()
            yaw = math.atan2(y2 - y1, x2 - x1)
            distance = math.hypot(x2 - x1, y2 - y1)
            steps = max(1, int(distance / max(path_step, map_msg.info.resolution)))

            for step in range(steps + 1):
                t = step / steps
                x = x1 + (x2 - x1) * t
                y = y1 + (y2 - y1) * t
                if not self.is_world_point_safe(x, y, map_msg, safe_grid):
                    rejected += 1
                    continue
                if step == 0:
                    connected = self.append_checked_pose(ros_path, x, y, yaw, map_msg, safe_grid)
                    if not connected:
                        disconnected_starts += 1
                        self.append_pose(ros_path, x, y, yaw)
                    continue

                self.append_checked_pose(ros_path, x, y, yaw, map_msg, safe_grid)

        if rejected:
            self.get_logger().warn(
                f"Dropped {rejected} sampled swath points outside /coverage/safe_map."
            )
        if disconnected_starts:
            self.get_logger().warn(
                f"Started {disconnected_starts} swath(s) without a safe transition; "
                "coverage points were kept, but execution should use Nav2 transitions."
            )

    def append_pose(self, ros_path, x, y, yaw):
        pose = PoseStamped()
        pose.header = ros_path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        ros_path.poses.append(pose)

    def create_marker_reset(self, header):
        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)
        return markers

    def append_swath_markers(self, swaths, markers, header, start_id):
        marker_id = start_id
        for i in range(swaths.size()):
            swath = swaths.at(i)
            start = swath.startPoint()
            end = swath.endPoint()

            marker = Marker()
            marker.header = header
            marker.ns = "f2c_swaths"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.035
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.1
            marker.color.a = 1.0

            p1 = Point(x=start.getX(), y=start.getY(), z=0.04)
            p2 = Point(x=end.getX(), y=end.getY(), z=0.04)
            marker.points = [p1, p2]

            markers.markers.append(marker)
            marker_id += 1

        return marker_id

    def append_checked_pose(self, ros_path, x, y, yaw, map_msg, safe_grid):
        if not ros_path.poses:
            self.append_pose(ros_path, x, y, yaw)
            return True

        prev = ros_path.poses[-1].pose.position
        if self.is_world_segment_safe(prev.x, prev.y, x, y, map_msg, safe_grid):
            self.append_pose(ros_path, x, y, yaw)
            return True

        if not bool(self.get_parameter("repair_unsafe_connections").value):
            self.get_logger().warn("Skipped unsafe F2C connection; grid repair disabled.")
            return False

        repair_points = self.plan_grid_transition(prev.x, prev.y, x, y, map_msg, safe_grid)
        if not repair_points:
            self.get_logger().warn("Skipped unsafe F2C connection; no safe grid transition found.")
            return False

        last_x = prev.x
        last_y = prev.y
        for repair_x, repair_y in repair_points[1:]:
            repair_yaw = math.atan2(repair_y - last_y, repair_x - last_x)
            self.append_pose(ros_path, repair_x, repair_y, repair_yaw)
            last_x = repair_x
            last_y = repair_y

        self.append_pose(ros_path, x, y, yaw)
        return True

    def create_swath_objective(self):
        objective_name = str(self.get_parameter("swath_objective").value)
        objectives = {
            "field_coverage": f2c.OBJ_FieldCoverage,
            "overlaps": f2c.OBJ_Overlaps,
            "swath_length": f2c.OBJ_SwathLength,
            "n_swath": f2c.OBJ_NSwath,
            "n_swath_modified": f2c.OBJ_NSwathModified,
        }

        if objective_name not in objectives:
            self.get_logger().warn(
                f"Unknown swath_objective '{objective_name}', using field_coverage."
            )
            objective_name = "field_coverage"

        return objectives[objective_name]()

    def append_f2c_path_to_ros_path(self, f2c_path, ros_path, map_msg):
        rejected = 0
        last_safe_point = None
        safe_grid = np.array(map_msg.data, dtype=np.int16).reshape(
            map_msg.info.height,
            map_msg.info.width,
        )

        for i in range(f2c_path.size()):
            state = f2c_path.getState(i)
            point = state.point
            yaw = state.angle

            x = point.getX()
            y = point.getY()

            if not self.is_world_point_safe(x, y, map_msg, safe_grid):
                rejected += 1
                continue

            self.append_checked_pose(ros_path, x, y, yaw, map_msg, safe_grid)
            last_safe_point = (x, y)

        if rejected:
            self.get_logger().warn(
                f"Dropped {rejected} F2C path states outside /coverage/safe_map."
            )

    def is_world_point_safe(self, x, y, msg, safe_grid):
        x_px, y_px = self.map_to_pixel(x, y, msg)
        if x_px is None or y_px is None:
            return False

        return safe_grid[y_px, x_px] == 0

    def is_world_segment_safe(self, x1, y1, x2, y2, msg, safe_grid):
        step_m = float(self.get_parameter("path_validation_step_m").value)
        distance = math.hypot(x2 - x1, y2 - y1)
        steps = max(1, int(distance / max(step_m, msg.info.resolution)))

        for i in range(steps + 1):
            t = i / steps
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            if not self.is_world_point_safe(x, y, msg, safe_grid):
                return False

        return True

    def plan_grid_transition(self, x1, y1, x2, y2, msg, safe_grid):
        start = self.map_to_pixel(x1, y1, msg)
        goal = self.map_to_pixel(x2, y2, msg)
        if start[0] is None or goal[0] is None:
            return []

        pixel_path = self.astar_grid_path(start, goal, safe_grid)
        if not pixel_path:
            return []

        return [self.pixel_to_map(px, py, msg) for px, py in pixel_path]

    def astar_grid_path(self, start, goal, safe_grid):
        if safe_grid[start[1], start[0]] != 0 or safe_grid[goal[1], goal[0]] != 0:
            return []

        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        ]
        if bool(self.get_parameter("allow_diagonal_repair").value):
            neighbors.extend([
                (-1, -1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
                (-1, 1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
            ])

        open_heap = [(0.0, start)]
        came_from = {}
        cost_so_far = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                return self.reconstruct_grid_path(came_from, current)

            for dx, dy, step_cost in neighbors:
                nx = current[0] + dx
                ny = current[1] + dy
                if ny < 0 or ny >= safe_grid.shape[0] or nx < 0 or nx >= safe_grid.shape[1]:
                    continue
                if safe_grid[ny, nx] != 0:
                    continue
                next_node = (nx, ny)
                new_cost = cost_so_far[current] + step_cost
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + math.hypot(goal[0] - nx, goal[1] - ny)
                    heapq.heappush(open_heap, (priority, next_node))
                    came_from[next_node] = current

        return []

    def reconstruct_grid_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def map_to_pixel(self, x, y, msg):
        resolution = msg.info.resolution
        origin = msg.info.origin
        yaw = self.quaternion_to_yaw(origin.orientation)

        dx = x - origin.position.x
        dy = y - origin.position.y
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

        x_px = int(local_x / resolution)
        y_px = int(local_y / resolution)

        if x_px < 0 or y_px < 0 or x_px >= msg.info.width or y_px >= msg.info.height:
            return None, None

        return x_px, y_px

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

    def publish_latest(self):
        if self.latest_path is not None:
            self.path_pub.publish(self.latest_path)
        if self.latest_swath_markers is not None:
            self.swath_marker_pub.publish(self.latest_swath_markers)


def main(args=None):
    rclpy.init(args=args)
    node = Fields2CoverPathGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
