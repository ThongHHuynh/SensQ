"""Launch the mission sequencer action server."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    docking_share = get_package_share_directory('my_robot_docking')
    default_database = os.path.join(
        docking_share,
        'config',
        'dock_database.yaml',
    )
    database = LaunchConfiguration('dock_database_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the simulation clock.',
        ),
        DeclareLaunchArgument(
            'dock_database_file',
            default_value=default_database,
            description='Dock database used to look up undock/dock targets.',
        ),
        Node(
            package='my_robot_mission',
            executable='mission_sequencer',
            name='mission_sequencer',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'dock_database_file': database,
                },
            ],
        ),
    ])
