# based on: https://automaticaddison.com/set-up-lidar-for-a-simulated-mobile-robot-in-ros-2/

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    gazeboDemoSharedDir = get_package_share_directory('gazebo_demo')
    worldFile = os.path.join(gazeboDemoSharedDir, 'world/diff_drive_robot.world')
    rvizConfigFile = os.path.join(get_package_share_directory('gazebo_demo'), 'rviz/gazebo_demo.rviz')

    ld.add_action(
        ExecuteProcess(
            cmd=['gazebo', '--verbose', worldFile],
            output='screen'
        )
    )

    ld.add_action(
        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0.5", "0", "0.25", "0", "0", "0", "chassis", "lidar_link"]
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
