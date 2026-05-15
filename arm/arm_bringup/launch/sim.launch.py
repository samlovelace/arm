from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="arm_hardware",
                executable="arm_hardware",
                output="screen",
            ),
            Node(
                package="arm_controller",
                executable="arm_controller",
                output="screen",
            ),
        ]
    )
