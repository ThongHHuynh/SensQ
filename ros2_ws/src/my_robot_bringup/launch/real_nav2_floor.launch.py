import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description_path = get_package_share_path("my_robot_description")
    robot_bringup_path = get_package_share_path("my_robot_bringup")

    urdf_path = os.path.join(robot_description_path, "urdf", "my_robot.urdf.xacro")
    controller_path = os.path.join(robot_bringup_path, "config", "my_robot_controller.yaml")
    default_nav2_params_path = os.path.join(robot_bringup_path, "config", "nav2_params.yaml")
    default_rviz_config_path = os.path.join(robot_description_path, "rviz", "urdf_config.rviz")
    default_map_path = os.path.join(
        os.getenv("ROS_WORKSPACE", "/home/tom/SensQ/ros2_ws"),
        "maps",
        "floor.yaml",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_path,
                " ",
                "use_mock_hardware:=false ",
                "serial_port:=",
                serial_port,
                " ",
                "baud:=",
                baud,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": False},
        ],
        output="screen",
    )

    controller_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=4.0,
        actions=[joint_state_broadcaster],
    )

    start_diff_drive_after_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive],
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_bringup_path, "launch", "lidar.launch.py"))
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": "false",
            "slam": "False",
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )

    nav2_group = GroupAction(
        actions=[
            SetRemap(src="/odom", dst="/diff_drive_controller/odom"),
            SetRemap(src="odom", dst="/diff_drive_controller/odom"),
            nav2_bringup,
        ]
    )

    delayed_nav2 = TimerAction(
        period=6.0,
        actions=[nav2_group],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyACM0",
                description="Serial port used by the mobile base hardware interface.",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="115200",
                description="Serial baud rate used by the mobile base hardware interface.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map_path,
                description="Map YAML file loaded by Nav2 map_server.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_nav2_params_path,
                description="Nav2 parameters file.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz with the robot description config.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config_path,
                description="RViz config path.",
            ),
            robot_state_publisher_node,
            controller_node,
            delayed_joint_state_broadcaster,
            start_diff_drive_after_joint_state_broadcaster,
            lidar_launch,
            delayed_nav2,
            rviz2_node,
        ]
    )
