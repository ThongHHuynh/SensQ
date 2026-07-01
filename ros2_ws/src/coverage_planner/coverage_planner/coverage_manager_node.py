#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class CoverageManagerNode(Node):
    def __init__(self):
        super().__init__("coverage_manager_node")

        self.declare_parameter("execute_coverage", False)
        self.declare_parameter("path_topic", "/coverage/path")
        
        self.execute_coverage = self.get_parameter("execute_coverage").value
        path_topic = self.get_parameter("path_topic").value

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE

        self.path_sub = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            qos_profile
        )

        if self.execute_coverage:
            self.navigator = BasicNavigator()
            self.get_logger().info("Coverage manager started. Waiting for path...")
        else:
            self.navigator = None
            self.get_logger().info("Coverage manager started. execute_coverage:=false (visualize only).")
            
        self.path_received = False

    def path_callback(self, msg):
        if self.path_received:
            return

        if not self.execute_coverage:
            self.get_logger().info(f"Received path with {len(msg.poses)} waypoints. Set execute_coverage:=true to navigate.")
            self.path_received = True
            return

        self.get_logger().info(f"Received dense path with {len(msg.poses)} waypoints. Extracting key corners...")
        self.path_received = True
        
        self.navigator.waitUntilNav2Active()
        
        # Extract sparse key poses (corners) to avoid overloading the global planner,
        # while still allowing us to use goThroughPoses which provides BT recoveries.
        key_poses = []
        if msg.poses:
            key_poses.append(msg.poses[0])
            for i in range(0, len(msg.poses) - 1):
                curr_z = msg.poses[i].pose.orientation.z
                next_z = msg.poses[i+1].pose.orientation.z
                if abs(curr_z - next_z) > 0.01:
                    key_poses.append(msg.poses[i])
            key_poses.append(msg.poses[-1])

        self.get_logger().info(f"Extracted {len(key_poses)} key poses. Navigating to start...")
        
        first_pose = key_poses[0]
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

def main(args=None):
    rclpy.init(args=args)
    node = CoverageManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()