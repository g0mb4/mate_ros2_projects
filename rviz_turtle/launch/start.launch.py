import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    urdfFile = os.path.join(get_package_share_directory('rviz_turtle'), 'urdf/robot.urdf')
    rvizConfigFile = os.path.join(get_package_share_directory('rviz_turtle'), 'rviz/rviz_turtle.rviz')

    with open(urdfFile, 'r') as f:
        robot_desc = f.read()

    ld.add_action(
        Node(
            package="turtlesim",
            executable="turtlesim_node"
        )
    )

    ld.add_action(
        Node(
            package="rviz_turtle",
            executable="pose_translator"
        )
    )

    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc}
            ],
            arguments=[urdfFile]
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output='screen',
            arguments=['-d', rvizConfigFile]
        )
    )

    return ld
