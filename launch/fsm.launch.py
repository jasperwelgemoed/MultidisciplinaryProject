# my_fsm_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="group03",
                executable="fsm_node",
                name="fsm_node",
                output="screen",
            )
        ]
    )
