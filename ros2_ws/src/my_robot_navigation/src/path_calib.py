#!/usr/bin/env python3

import math

import rclpy
import tf_transformations
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Path


def create_pose_stamped(
    navigator: BasicNavigator,
    x: float,
    y: float,
    yaw: float,
) -> PoseStamped:
    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(
        0.0,
        0.0,
        yaw,
    )

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose


def create_premade_path(navigator: BasicNavigator) -> Path:
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = navigator.get_clock().now().to_msg()

    # Premade path: x, y, yaw in degrees.
    path_points = [
        (0.0, 0.0, 0.0),
        (0.25, 0.0, 0.0),
        (0.50, 0.0, 0.0),
        (0.75, 0.0, 0.0),
        (1.00, 0.0, 0.0),
        (0.75, 0.0, 180.0),
        (0.50, 0.0, 180.0),
        (0.25, 0.0, 180.0),
        (0.00, 0.0, 180.0),
    ]

    for x, y, yaw_degrees in path_points:
        pose = create_pose_stamped(
            navigator,
            x,
            y,
            math.radians(yaw_degrees),
        )

        # Use the same timestamp and frame throughout the path.
        pose.header = path.header
        path.poses.append(pose)

    return path


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set AMCL's initial estimate.
    initial_pose = create_pose_stamped(
        navigator,
        0.0,
        0.0,
        0.0,
    )
    navigator.setInitialPose(initial_pose)

    print("Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    print("Nav2 is active.")

    path = create_premade_path(navigator)

    print(f"Sending path with {len(path.poses)} poses...")

    navigator.followPath(
        path,
        controller_id="FollowPath",
        goal_checker_id="general_goal_checker",
    )

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()

        if feedback is not None:
            print(
                f"Distance remaining: "
                f"{feedback.distance_to_goal:.2f} m, "
                f"speed: {feedback.speed:.2f} m/s"
            )

    result = navigator.getResult()
    print(f"FollowPath result: {result}")

    navigator.destroyNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()