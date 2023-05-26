"""
Launch file for starting telecom subsystem at base station.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for base station telecom system."""
    img_width, img_height = 1280, 720

    isaac_package = 'isaac_ros_h264_decoder'
    launch_args = {'input_width': img_width, 'input_height': img_height}
    # decoder = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory(isaac_package),
    #         'launch', 'isaac_ros_h264_decoder.launch.py'
    #     )]), launch_arguments=launch_args.items()
    # )

    cam_sub = Node(
        package='telecom',
        executable='cam_sub',
        output='screen',
        remappings=[
            ('/image_cv_compressed', '/image_uncompressed')
        ]
    )

    return LaunchDescription([
        # decoder,
        cam_sub,
    ])
