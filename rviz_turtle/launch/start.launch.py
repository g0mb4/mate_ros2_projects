from launch import LaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package="turtlesim",
        executable="turtlesim_node"
    ))

    ld.add_action(Node(
        package="rviz_turtle",
        executable="pose_translator"
    ))

    ld.add_action(Node(
        package="rviz2",
        executable="rviz2",
        output='screen',
        arguments=['-d', [ThisLaunchFileDir(), '/../rviz/rviz_turtle.rviz']],
    ))

    return ld
