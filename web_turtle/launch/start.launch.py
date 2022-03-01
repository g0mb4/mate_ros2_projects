import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
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
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rosbridge_server'),
                    'launch/rosbridge_websocket_launch.xml'
                    )
                )
        ) 
    )

    ld.add_action(
        ExecuteProcess(
            cmd=[
                'python3',
                '-u',
                os.path.join(
                        get_package_share_directory('web_turtle'),
                        'scripts/server.py'
                    )
            ],
            name='webserver',
            output='screen'
        )
    )

    return ld