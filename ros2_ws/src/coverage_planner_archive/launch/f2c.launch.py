from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("coverage_planner_archive")
    config_file = os.path.join(pkg_dir, "config", "f2c.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    execute_coverage = LaunchConfiguration("execute_coverage")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("execute_coverage", default_value="false"),
        Node(
            package="coverage_planner_archive",
            executable="map_processor_node",
            name="map_processor_node",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="coverage_planner_archive",
            executable="open_coverage_path",
            name="open_coverage_path",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="coverage_planner_archive",
            executable="f2c_path_gen_node",
            name="f2c_path_gen_node",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="coverage_planner_archive",
            executable="coverage_visualizer_node",
            name="coverage_visualizer_node",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
    ])
