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

    cam1_pub = Node(
        package='telecom',
        executable='cam_pub',
        output='screen',
        name='cam1_pub',
    )

    cam2_pub = Node(
        package='telecom',
        executable='cam_pub',
        output='screen',
        name='cam2_pub',
    )

    cam3_pub = Node(
        package='telecom',
        executable='cam_pub',
        output='screen',
        name='cam3_pub',
    )

    cam4_pub = Node(
        package='telecom',
        executable='cam_pub',
        output='screen',
        name='cam4_pub',
    )

    return LaunchDescription([
        zed_pub,
        cam1_pub,
        cam2_pub,
        cam3_pub,
        cam4_pub,
    ])
