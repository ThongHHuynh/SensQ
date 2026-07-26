from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("test_coverage")
    default_parameters = os.path.join(package_share, "config", "coverage.yaml")
    default_rviz_config = os.path.join(package_share, "rviz", "coverage.rviz")

    parameters_file = LaunchConfiguration("params_file")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    auto_generate = ParameterValue(
        LaunchConfiguration("auto_generate"), value_type=bool
    )
    auto_start = ParameterValue(LaunchConfiguration("auto_start"), value_type=bool)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_parameters,
                description="Coverage planner and executor parameter file",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("auto_generate", default_value="false"),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start the coverage-only RViz view",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz configuration file",
            ),
            Node(
                package="test_coverage",
                executable="coverage_planner",
                name="coverage_planner",
                output="screen",
                parameters=[
                    parameters_file,
                    {
                        "use_sim_time": use_sim_time,
                        "auto_generate": auto_generate,
                    },
                ],
            ),
            Node(
                package="test_coverage",
                executable="coverage_executor",
                name="coverage_executor",
                output="screen",
                parameters=[
                    parameters_file,
                    {
                        "use_sim_time": use_sim_time,
                        "auto_start": auto_start,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="coverage_rviz",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
