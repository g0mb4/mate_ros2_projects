from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="dummy_camera",
            executable="dummy_camera",
            name="camera",
            remappings=[
                ("dummy_camera/image", "image")
            ],
            parameters=[
                {"files": ["/home/gmb/asd.png", "/home/gmb/asd2.png"]},
                {"fps": 1}
            ]
        )
    )

    return ld
