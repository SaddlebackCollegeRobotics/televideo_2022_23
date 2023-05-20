"""
Launch file for starting telecom subsystem on rover.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for rover telecom system."""
    img_width, img_height = 1920, 1200

    isaac_package = 'isaac_ros_h264_encoder'
    launch_args = {'input_width': img_width, 'input_height': img_height}
    encoder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(isaac_package),
            'launch', 'isaac_ros_h264_encoder.launch.py'
        )]), launch_arguments=launch_args.items()
    )

    zed_pub = Node(
        package='telecom',
        executable='zed_pub',
        output='screen',
    )

    return LaunchDescription([
        encoder,
        zed_pub,
    ])
