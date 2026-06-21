from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
import os
def generate_launch_description():
    robot_description_path = get_package_share_path('my_robot_description')

    rviz_config_path = os.path.join(robot_description_path, 'rviz', 'urdf_config.rviz')

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',rviz_config_path]
    )
    ld = LaunchDescription()
    ld.add_action(rviz2_node)
    return ld