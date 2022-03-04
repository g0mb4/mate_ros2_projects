from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="turtlesim",
            executable="turtlesim_node"
        )
    )

    ld.add_action(
        Node(
            package="qt_turtle",
            executable="qt_turtle"
        )
    )

    return ld
