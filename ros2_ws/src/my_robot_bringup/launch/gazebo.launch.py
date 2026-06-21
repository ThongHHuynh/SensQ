from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def serial_available(path ="/dev/ttyUSB0"):
    return os.path.exists(path) and os.access(path, os.R_OK | os.W_OK)

def generate_launch_description():
    port = "/dev/ttyUSB0"
    # Gazebo launch should not talk to real hardware even if a serial device exists.
    use_mock = True

    robot_description_path = get_package_share_path('my_robot_description')
    robot_bringup_path = get_package_share_path('my_robot_bringup')

    urdf_path = os.path.join(robot_description_path, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(robot_description_path, 'rviz', 'urdf_config.rviz')
    controller_path = os.path.join(robot_bringup_path, 'config', 'my_robot_controller.yaml')

    gazebo_config_path = os.path.join(robot_bringup_path, 'config', 'gazebo_bridge.yaml')
    map_path = os.path.join(robot_description_path, 'worlds', 'maze.sdf')

    slam_toolbox_path = os.path.join(robot_bringup_path, 'config', 'slam_toolbox.yaml')

    robot_description = ParameterValue(Command(['xacro ', urdf_path,' ',
                                                'use_mock_hardware:=', 'true' if use_mock else 'false', ' ',
                                                'serial_port:=', port,' ',
                                                'baud:=','115200']), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},  # require exact param name
            {'use_sim_time': True},
        ]
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", map_path]}.items(),
    )
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "my_robot",
            "-topic",
            "/robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.25",
        ],
        output="screen",
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': gazebo_config_path}]
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
    # ld.add_action(controller_node)
    # ld.add_action(joint_state_broadcaster)
    # ld.add_action(diff_drive)
    # ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz2_node)
    ld.add_action(gz_sim)
    ld.add_action(spawn_entity)
    ld.add_action(ros_gz_bridge)
    ld.add_action(slam_node)

    return ld
