#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


class CoverageVisualizerNode(Node):
    def __init__(self):
        super().__init__("coverage_visualizer_node")

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE

        self.path_sub = self.create_subscription(
            Path,
            "/coverage/path",
            self.path_callback,
            qos_profile=qos_profile
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/coverage/waypoint_markers",
            qos_profile=qos_profile
        )
        
        self.latest_markers = None
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

    def publish_latest(self):
        if self.latest_markers is not None:
            self.marker_pub.publish(self.latest_markers)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()