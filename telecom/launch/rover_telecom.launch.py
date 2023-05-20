"""
Launch file for starting telecom subsystem on rover.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for rover telecom system."""
    zed_pub = Node(
        package='telecom',
        executable='zed_pub',
        output='screen',
    )

    return LaunchDescription([
        zed_pub,
    ])
