import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    rosbridgeSharedDir = get_package_share_directory('rosbridge_server')
    rosbridgeLaunchFile = os.path.join(rosbridgeSharedDir, 'launch/rosbridge_websocket_launch.xml')

    webServerSharedDir = get_package_share_directory('web_turtle')
    webServerScriptFile = os.path.join(webServerSharedDir, 'scripts/server.py')

    ld.add_action(
        Node(
            package="turtlesim",
            executable="turtlesim_node"
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                rosbridgeLaunchFile
            )
        )
    )

    ld.add_action(
        ExecuteProcess(
            cmd=['python3', '-u', webServerScriptFile],
            name='webserver',
            output='screen'
        )
    )

    return ld