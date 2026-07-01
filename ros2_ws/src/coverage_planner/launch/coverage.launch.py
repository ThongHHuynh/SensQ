from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    execute_coverage = LaunchConfiguration("execute_coverage")
    clearance_m = LaunchConfiguration("clearance_m")
    spacing_m = LaunchConfiguration("spacing_m")

    coverage_node = Node(
        package="coverage_planner",
        executable="coverage_node",
        name="coverage_planner",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "execute_coverage": execute_coverage,
                "clearance_m": clearance_m,
                "spacing_m": spacing_m,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("execute_coverage", default_value="false"),
            DeclareLaunchArgument("clearance_m", default_value="0.35"),
            DeclareLaunchArgument("spacing_m", default_value="0.30"),
            coverage_node,
        ]
    )
