"""Launch AprilTag detection, docking server, and velocity arbitration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('my_robot_docking')
    server_config = os.path.join(
        package_share,
        'config',
        'docking_config.yaml',
    )
    detector_config = os.path.join(
        package_share,
        'config',
        'apriltag.yaml',
    )
    database = os.path.join(
        package_share,
        'config',
        'dock_database.yaml',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_apriltag = LaunchConfiguration('start_apriltag')
    detector_qos = LaunchConfiguration('detector_qos')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the simulation clock.',
        ),
        DeclareLaunchArgument(
            'start_apriltag',
            default_value='true',
            description='Start apriltag_ros for the color camera.',
        ),
        DeclareLaunchArgument(
            'detector_qos',
            default_value='sensor_data',
            description='QoS preset used by the AprilTag camera subscriber.',
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            condition=IfCondition(start_apriltag),
            remappings=[
                ('image_rect', '/camera/color/image_raw'),
                ('camera_info', '/camera/color/camera_info'),
            ],
            parameters=[
                detector_config,
                {
                    'use_sim_time': use_sim_time,
                    'qos_profile': detector_qos,
                },
            ],
        ),
        Node(
            package='my_robot_docking',
            executable='velocity_arbiter',
            name='velocity_arbiter',
            output='screen',
            parameters=[
                server_config,
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='my_robot_docking',
            executable='docking_server',
            name='docking_server',
            output='screen',
            parameters=[
                server_config,
                {
                    'use_sim_time': use_sim_time,
                    'dock_database_file': database,
                },
            ],
        ),
    ])
