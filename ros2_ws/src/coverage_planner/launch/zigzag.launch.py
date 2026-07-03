from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("coverage_planner")
    config_file = os.path.join(pkg_dir, "config", "coverage.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    execute_coverage = LaunchConfiguration("execute_coverage")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("execute_coverage", default_value="false"),
        Node(
            package="coverage_planner",
            executable="coverage_manager_node",
            name="coverage_manager_node",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time, "execute_coverage": execute_coverage}],
        ),
        Node(
            package="coverage_planner",
            executable="map_processor_node",
            name="map_processor_node",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="coverage_planner",
            executable="path_generator_node",
            name="path_generator_node",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="coverage_planner",
            executable="coverage_visualizer_node",
            name="coverage_visualizer_node",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
        # Node(
        #     package="coverage_planner",
        #     executable="coverage_database_node",
        #     name="coverage_database_node",
        #     output="screen",
        #     parameters=[config_file],
        # ),
        # Node(
        #     package="coverage_planner",
        #     executable="nav2_executor_node",
        #     name="nav2_executor_node",
        #     output="screen",
        #     parameters=[config_file],
        # ),
    ])