#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Point32, PolygonStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class OpenCoveragePathNode(Node):
    def __init__(self):
        super().__init__("open_coverage_path")

        self.declare_parameter("map_topic", "/coverage/safe_map")
        self.declare_parameter("boundary_topic", "/coverage/opennav_boundary")
        self.declare_parameter("wkt_topic", "/coverage/opennav_wkt")
        self.declare_parameter("marker_topic", "/coverage/opennav_polygons")
        self.declare_parameter("contour_simplification_px", 1.0)
        self.declare_parameter("min_region_area_m2", 0.20)
        self.declare_parameter("min_hole_area_m2", 0.02)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.map_callback,
            qos,
        )
        self.boundary_pub = self.create_publisher(
            PolygonStamped,
            self.get_parameter("boundary_topic").value,
            qos,
        )
        self.wkt_pub = self.create_publisher(
            String,
            self.get_parameter("wkt_topic").value,
            qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("marker_topic").value,
            qos,
        )

        self.latest_boundary = None
        self.latest_wkt = None
        self.latest_markers = None
        self.has_run = False
        self.timer = self.create_timer(1.0, self.publish_latest)

        self.get_logger().info("Open coverage polygon converter started.")

    def map_callback(self, msg):
        if self.has_run:
            return

        self.has_run = True
        result = self.safe_map_to_polygons(msg)
        if result is None:
            self.get_logger().warn("No valid safe free-space polygon found.")
            return

        exterior, holes, wkt = result

        boundary = PolygonStamped()
        boundary.header.frame_id = msg.header.frame_id or "map"
        boundary.header.stamp.sec = 0
        boundary.header.stamp.nanosec = 0
        boundary.polygon.points = [
            Point32(x=float(x), y=float(y), z=0.0)
            for x, y in exterior
        ]

        wkt_msg = String()
        wkt_msg.data = wkt

        markers = self.build_markers(boundary.header, exterior, holes)

        self.latest_boundary = boundary
        self.latest_wkt = wkt_msg
        self.latest_markers = markers

        self.publish_latest()
        self.get_logger().info(
            f"Published OpenNav polygon with {len(exterior)} boundary points "
            f"and {len(holes)} holes."
        )

    def safe_map_to_polygons(self, msg):
        simplify_px = float(self.get_parameter("contour_simplification_px").value)
        min_region_area_m2 = float(self.get_parameter("min_region_area_m2").value)
        min_hole_area_m2 = float(self.get_parameter("min_hole_area_m2").value)
        min_region_area_px = min_region_area_m2 / (msg.info.resolution ** 2)
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
        outer_indices = [
            i for i, contour in enumerate(contours)
            if hierarchy[i][3] == -1 and cv2.contourArea(contour) >= min_region_area_px
        ]
        if not outer_indices:
            return None

        regions = []
        for outer_idx in sorted(
            outer_indices,
            key=lambda i: cv2.contourArea(contours[i]),
            reverse=True,
        ):
            exterior = self.contour_to_points(contours[outer_idx], msg, simplify_px)
            if len(exterior) < 4:
                continue

            holes = []
            hole_idx = hierarchy[outer_idx][2]
            while hole_idx != -1:
                hole = contours[hole_idx]
                if cv2.contourArea(hole) >= min_hole_area_px:
                    points = self.contour_to_points(hole, msg, simplify_px)
                    if len(points) >= 4:
                        holes.append(points)
                hole_idx = hierarchy[hole_idx][0]

            regions.append((exterior, holes))

        if not regions:
            return None

        # Keep /coverage/opennav_boundary as one PolygonStamped for OpenNav
        # compatibility, but publish all safe regions in WKT for F2C.
        exterior, holes = regions[0]
        wkt = self.polygons_to_wkt(regions)
        if len(regions) > 1:
            self.get_logger().warn(
                f"Found {len(regions)} disconnected safe regions; "
                "published the largest boundary and all regions in WKT."
            )

        return exterior, holes, wkt

    def contour_to_points(self, contour, msg, simplify_px):
        if simplify_px > 0.0:
            contour = cv2.approxPolyDP(contour, simplify_px, True)

        points = []
        for p in contour[:, 0, :]:
            x, y = self.pixel_to_map(int(p[0]), int(p[1]), msg)
            if points and math.isclose(points[-1][0], x) and math.isclose(points[-1][1], y):
                continue
            points.append((x, y))

        if points and points[0] != points[-1]:
            points.append(points[0])

        return points

    def polygons_to_wkt(self, regions):
        polygon_bodies = []
        for exterior, holes in regions:
            rings = [self.points_to_wkt_ring(exterior)]
            rings.extend(self.points_to_wkt_ring(hole) for hole in holes)
            polygon_bodies.append(f"({','.join(rings)})")

        if len(polygon_bodies) == 1:
            return f"POLYGON {polygon_bodies[0]}"

        return f"MULTIPOLYGON ({','.join(polygon_bodies)})"

    def points_to_wkt_ring(self, points):
        return "(" + ",".join(f"{x:.6f} {y:.6f}" for x, y in points) + ")"

    def build_markers(self, header, exterior, holes):
        markers = MarkerArray()

        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        markers.markers.append(
            self.line_marker(header, 0, "opennav_boundary", exterior, 0.0, 1.0, 0.2)
        )

        for i, hole in enumerate(holes, start=1):
            markers.markers.append(
                self.line_marker(header, i, "opennav_holes", hole, 1.0, 0.1, 0.1)
            )

        return markers

    def line_marker(self, header, marker_id, namespace, points, r, g, b):
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.points = [Point(x=float(x), y=float(y), z=0.03) for x, y in points]
        return marker

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
        if self.latest_boundary is not None:
            self.boundary_pub.publish(self.latest_boundary)
        if self.latest_wkt is not None:
            self.wkt_pub.publish(self.latest_wkt)
        if self.latest_markers is not None:
            self.marker_pub.publish(self.latest_markers)


def main(args=None):
    rclpy.init(args=args)
    node = OpenCoveragePathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
