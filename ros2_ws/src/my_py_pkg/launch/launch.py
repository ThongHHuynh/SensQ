from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    number_publisher = Node(
        package="my_py_pkg",
        executable="number_publisher"
    )
    number_counter = Node(
        package="my_py_pkg",
        executable="hardware_status_publisher"
    )
    
    ld.add_action(number_publisher)
    ld.add_action(number_counter)
    return ld