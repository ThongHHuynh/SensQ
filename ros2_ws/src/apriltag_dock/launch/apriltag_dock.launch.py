"""
Launch file for the apriltag_dock package.

Launches:
  1. ``apriltag_ros`` AprilTagNode — detects tags and broadcasts TF
  2. ``dock_controller_node`` — precision docking state machine

The apriltag_ros node subscribes to the camera image and camera_info
topics, detects AprilTag 36h11 markers, and broadcasts each tag pose
as a TF frame (e.g. ``tag_0``, ``tag_1``).

Usage:
    ros2 launch apriltag_dock apriltag_dock.launch.py use_sim_time:=true
    ros2 launch apriltag_dock apriltag_dock.launch.py use_sim_time:=true target_tag_id:=1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('apriltag_dock')
    params_file = os.path.join(pkg_dir, 'config', 'apriltag_dock_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # ── Launch arguments ────────────────────────────────────────────
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock',
        ),

        # ── AprilTag detection (system apriltag_ros package) ───────────
        # The node subscribes to image_rect and camera_info, detects
        # AprilTags, and broadcasts TF transforms for each tag.
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            namespace='',
            output='screen',
            remappings=[
                ('image_rect', '/camera/color/image_raw'),
                ('camera_info', '/camera/color/camera_info'),
            ],
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),

        # ── Docking controller ─────────────────────────────────────────
        Node(
            package='apriltag_dock',
            executable='dock_controller_node',
            name='dock_controller_node',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
