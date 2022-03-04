import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    gazeboDemoSharedDir = get_package_share_directory('gazebo_demo')
    worldFile = os.path.join(gazeboDemoSharedDir, 'world/diff_drive_robot.world')

    ld.add_action(
        ExecuteProcess(
            cmd=['gazebo', '--verbose', worldFile],
            output='screen'
        )
    )

    return ld