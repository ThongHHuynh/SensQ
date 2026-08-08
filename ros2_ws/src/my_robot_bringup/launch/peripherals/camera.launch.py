from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_bringup',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])