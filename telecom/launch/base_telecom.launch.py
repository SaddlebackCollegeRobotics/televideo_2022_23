"""
Launch file for starting telecom subsystem at base station.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for base station telecom system."""
    cam_sub = Node(
        package='telecom',
        executable='cam_sub',
        output='screen',
    )

    return LaunchDescription([
        cam_sub,
    ])
