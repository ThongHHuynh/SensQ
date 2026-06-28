from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def serial_available(path ="/dev/ttyACM0"):
    return os.path.exists(path) and os.access(path, os.R_OK | os.W_OK)

def generate_launch_description():
    port = "/dev/ttyACM0"
    use_mock = not serial_available(port)

    robot_description_path = get_package_share_path('my_robot_description')
    robot_bringup_path = get_package_share_path('my_robot_bringup')

    #urdf_path = os.path.join(robot_description_path, 'urdf', 'my_robot.urdf.xacro')
    urdf_path = os.path.join(robot_description_path, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(robot_description_path, 'rviz', 'lidar_config.rviz')
    controller_path = os.path.join(robot_bringup_path, 'config', 'my_robot_controller.yaml')
    slam_toolbox_path = os.path.join(robot_bringup_path, 'config', 'slam_toolbox.yaml')

    robot_description = ParameterValue(Command(['xacro ', urdf_path,' ',
                                                'use_mock_hardware:=', 'true' if use_mock else 'false', ' ',
                                                'serial_port:=', port,' ',
                                                'baud:=','115200']), value_type=str)
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_path, 'launch', 'lidar.launch.py')
        )
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}] #require exact param name
    )
    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui"
    # )

    controller_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_path],
        remappings=[('/controller_manager/robot_description','/robot_description'),
                    ('/diff_drive_controller/cmd_vel_unstamped','/cmd_vel')]
    )
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    diff_drive = Node(
        package='controller_manager',
        executable= 'spawner',
        arguments=['diff_drive_controller'],
        #remappings=[('/diff_drive_controller/cmd_vel','/cmd_vel')]
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',rviz_config_path]
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name ='slam_toolbox', 
        parameters=[slam_toolbox_path],
        remappings=[('/odom', '/diff_drive_controller/odom')],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_node)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(diff_drive)
    # ld.add_action(joint_state_publisher_gui_node)
    # ld.add_action(rviz2_node)
    ld.add_action(lidar_launch)
    ld.add_action(slam_node)

    return ld
