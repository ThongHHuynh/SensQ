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
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    lidar_serial_baudrate = LaunchConfiguration("lidar_serial_baudrate")

    map_yaml = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    use_rviz = LaunchConfiguration("use_rviz")
    start_camera = LaunchConfiguration("start_camera")
    camera_sensor_id = LaunchConfiguration("camera_sensor_id")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_fps = LaunchConfiguration("camera_fps")
    camera_image_topic = LaunchConfiguration("camera_image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    camera_info_url = LaunchConfiguration("camera_info_url")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description_path = get_package_share_path("my_robot_description")
    robot_bringup_path = get_package_share_path("my_robot_bringup")
    robot_navigation_path = get_package_share_path("my_robot_navigation")
    robot_docking_path = get_package_share_path("my_robot_docking")
    real_dock_database = os.path.join(
            robot_docking_path,
            'config',
            'dock_database.yaml',
        )
    
    robot_camera_path = get_package_share_path("csi_camera")

    urdf_path = os.path.join(robot_description_path, "urdf", "my_robot.urdf.xacro")
    controller_path = os.path.join(robot_bringup_path, "config", "my_robot_controller.yaml")
    default_nav2_params_path = os.path.join(robot_navigation_path, "config", "nav2_config.yaml")
    default_rviz_config_path = os.path.join(robot_navigation_path, "rviz", "navigation_config.rviz")
    default_cam_calibration_path = os.path.join(robot_camera_path, "config", "camera.yaml")
    ekf_path = os.path.join(robot_navigation_path, "config", "ekf.yaml")
    default_map_path = os.path.join(
        os.getenv("ROS_WORKSPACE", "/home/tom/SensQ/ros2_ws"),
        "maps",
        "floor_v1.yaml",
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
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel_out"),
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

    delayed_joint_state_broadcaster = TimerAction(
        period=1.0,
        actions=[joint_state_broadcaster],
    )

    diff_drive_spawner = Node(
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

    start_diff_drive_after_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive_spawner],
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_bringup_path, "launch", "peripherals", "lidar.launch.py")),
        launch_arguments={
            "serial_port": lidar_serial_port,
            "serial_baudrate": lidar_serial_baudrate,
        }.items(),
    )
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("tm_imu"), "launch", "imu.launch.py")
        )
    )
    camera_node = Node(
        package="csi_camera",
        executable="camera_node",
        name="jetson_csi_camera",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "sensor_id": ParameterValue(camera_sensor_id, value_type=int),
                "width": ParameterValue(camera_width, value_type=int),
                "height": ParameterValue(camera_height, value_type=int),
                "fps": ParameterValue(camera_fps, value_type=int),
                "topic_name": camera_image_topic,
                "camera_info_topic": camera_info_topic,
                "camera_info_url": camera_info_url,
                "frame_id": camera_frame_id,
            }
        ],
        condition=IfCondition(start_camera),
    )
#EKF    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_path]
    )

#NAVIGATION
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": nav2_params_file,
            "use_sim_time": "false",
            "slam": "False",
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )


    start_localization_after_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[
                ekf_node,
                TimerAction(
                    period=2.0,
                    actions=[nav2_bringup],
                ),
            ],
        )
    )

    docking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_docking_path, "launch", "docking.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "start_apriltag": "true",
            "detector_qos": "sensor_data",
            "image_topic": camera_image_topic,
            "camera_info_topic": camera_info_topic,
            'dock_database_file': real_dock_database,
        }.items(),
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
                "lidar_serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial port used by the SLLidar device.",
            ),
            DeclareLaunchArgument(
                "lidar_serial_baudrate",
                default_value="460800",
                description="Serial baud rate used by the SLLidar device.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map_path,
                description="Map YAML file loaded by Nav2 map_server.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params_path,
                description="Nav2 parameters file.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="False",
                description="Start RViz with the robot description config.",
            ),
            DeclareLaunchArgument(
                "start_camera",
                default_value="True",
                description="Start the physical NVIDIA CSI camera driver.",
            ),
            DeclareLaunchArgument(
                "camera_sensor_id",
                default_value="0",
                description="nvarguscamerasrc sensor ID.",
            ),
            DeclareLaunchArgument(
                "camera_width",
                default_value="640",
                description="CSI image width; must match camera calibration.",
            ),
            DeclareLaunchArgument(
                "camera_height",
                default_value="480",
                description="CSI image height; must match camera calibration.",
            ),
            DeclareLaunchArgument(
                "camera_fps",
                default_value="30",
                description="CSI camera frame rate.",
            ),
            DeclareLaunchArgument(
                "camera_image_topic",
                default_value="/camera/image_raw",
                description="CSI image topic consumed by AprilTag.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera_info",
                description="CSI CameraInfo topic consumed by AprilTag.",
            ),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value=default_cam_calibration_path,
                description="file:// URL for matching CSI calibration YAML.",
            ),
            DeclareLaunchArgument(
                "camera_frame_id",
                default_value="camera_optical_frame",
                description="Optical TF frame used by CSI image headers.",
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
            start_localization_after_diff_drive,
            lidar_launch,
            imu_launch,
            camera_node,
            docking,
            rviz2_node,
        ]
    )
