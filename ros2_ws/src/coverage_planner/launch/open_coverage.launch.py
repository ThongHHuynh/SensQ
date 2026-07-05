from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("coverage_planner")
    config_file = os.path.join(pkg_dir, "config", "open_coverage.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="coverage_planner",
            executable="map_processor_node",
            name="map_processor_node",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="coverage_planner",
            executable="open_coverage_path",
            name="open_coverage_path",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
    ])
