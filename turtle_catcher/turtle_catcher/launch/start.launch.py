from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    tolerance = 0.2

    ld.add_action(
        Node(
            package="turtlesim",
            executable="turtlesim_node"
        )
    )

    ld.add_action(
        Node(
            package="turtle_catcher",
            executable="turtle1_controller",
            parameters=[
                {"P_distance": 0.5},
                {"P_angle": 6.0},
                {"tolerance": tolerance},
            ]
        )
    )

    ld.add_action(
        Node(
            package="turtle_catcher",
            executable="game_master",
            parameters=[
                {"tolerance": tolerance}
            ]
        )
    )

    return ld
