from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("coverage_planner_archive")
    config_file = os.path.join(pkg_dir, "config", "production_coverage.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    path_generator = LifecycleNode(
        package="coverage_planner_archive",
        executable="production_path_gen",
        name="production_path_gen",
        namespace="",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
    )
    executor = LifecycleNode(
        package="coverage_planner_archive",
        executable="coverage_executor_node",
        name="coverage_executor_node",
        namespace="",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
    )

    managed_nodes = [path_generator, executor]
    configure_events = [
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        )
        for node in managed_nodes
    ]
    activate_handlers = [
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            )
        )
        for node in managed_nodes
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # Map preprocessor (standard node, always runs).
        Node(
            package="coverage_planner_archive",
            executable="map_processor_node",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),

        # Path generator (lifecycle node).
        path_generator,

        # Visualizer.
        Node(
            package="coverage_planner_archive",
            executable="coverage_visualizer_node",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),

        # Executor (lifecycle node).
        executor,

        *activate_handlers,
        *configure_events,
    ])
