"""Launch the complete Gazebo navigation and docking simulation."""

from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def serial_available(path ="/dev/ttyUSB0"):
    return os.path.exists(path) and os.access(path, os.R_OK | os.W_OK)

def generate_launch_description():
    port = "/dev/ttyUSB0"
    # Gazebo launch should not talk to real hardware even if a serial device exists.
    use_mock = True

    # Isolate this run so a stale Gazebo server cannot receive its spawn request.
    gazebo_partition = f"my_robot_{os.getpid()}"
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description_path = get_package_share_path('my_robot_description')
    robot_bringup_path = get_package_share_path('my_robot_bringup')
    robot_navigation_path = get_package_share_path('my_robot_navigation')
    docking_path = get_package_share_path('my_robot_docking')

    urdf_path = os.path.join(robot_description_path, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(robot_navigation_path, 'rviz', 'navigation_config.rviz')
    controller_path = os.path.join(robot_bringup_path, 'config', 'my_robot_controller.yaml')

    gazebo_config_path = os.path.join(robot_bringup_path, 'config', 'gazebo_bridge.yaml')
    world_path = os.path.join(robot_description_path, 'worlds', 'maze_apriltags.sdf')
    nav_map_path = '/home/tom/maps/simple_maze.yaml'

    # Gazebo converts package:// mesh URIs to model:// URIs. Both the legacy
    # Ignition and current Gazebo variable must contain the parent directory
    # of the package share directory for those URIs to resolve.
    gazebo_resource_entries = [str(robot_description_path.parent)]
    for variable in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'):
        for entry in os.environ.get(variable, '').split(os.pathsep):
            if entry and entry not in gazebo_resource_entries:
                gazebo_resource_entries.append(entry)
    gazebo_resource_path = os.pathsep.join(gazebo_resource_entries)

    #slam_toolbox_path = os.path.join(robot_bringup_path, 'config', 'slam_toolbox.yaml')
    nav2_params = os.path.join(robot_navigation_path, 'config', 'sim_nav2_config.yaml')
    simulation_ekf_path = os.path.join(
        robot_navigation_path, 'config', 'sim-ekf.yaml'
    )
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
        condition=IfCondition(use_rviz),
    )
    
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world_path]}.items(),
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": [" -s -r -v 4 ", world_path]}.items(),
        condition=IfCondition(headless),
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
    delayed_spawn_entity = TimerAction(
        period=1.0,
        actions=[spawn_entity],
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': gazebo_config_path}],
        # The EKF is the sole odom -> base_footprint TF publisher.
        remappings=[
            ('/tf', '/gazebo/raw_tf'),
            ('/cmd_vel', '/cmd_vel_out'),
        ],
        output='screen',
    )

    docking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(docking_path, 'launch', 'docking.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'start_apriltag': 'true',
            'detector_qos': 'system_default',
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
        }.items(),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[simulation_ekf_path],
        output='screen',
    )

    delayed_ekf = TimerAction(
        period=2.0,
        actions=[ekf_node],
    )

    nav2_dir = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "launch",
                    "bringup_launch.py",
                )
            ], 
        ),
        launch_arguments = {
                'map': nav_map_path,
                'use_sim_time': 'true',
                'slam': 'False',
                'params_file': nav2_params,
                'autostart': 'true',
                'use_composition': 'False'}.items(),
    )

    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2_dir],
    )



    ld = LaunchDescription()
    ld.add_action(
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gazebo_resource_path)
    )
    ld.add_action(
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", gazebo_resource_path)
    )
    ld.add_action(SetEnvironmentVariable("IGN_PARTITION", gazebo_partition))
    ld.add_action(LogInfo(msg=f"Gazebo transport partition: {gazebo_partition}"))
    ld.add_action(
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run the Gazebo server without its GUI.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz.',
        )
    )
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz2_node)
    ld.add_action(gz_sim_gui)
    ld.add_action(gz_sim_headless)
    ld.add_action(delayed_spawn_entity)
    ld.add_action(ros_gz_bridge)
    ld.add_action(docking)
    ld.add_action(delayed_ekf)
    ld.add_action(delayed_nav2)

    return ld
