"""ROS2 Node for publishing compressed and uncompressed images"""
import json
import os
from typing import NamedTuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np


class Resolution(NamedTuple):
    """Default (single) camera resolution."""
    width: int = 2208
    height: int = 1242


class ZedPublisher(Node):
    """Publishes compressed and uncompressed image feeds"""

    def __init__(self, fps: int = 24):
        super().__init__('zed_pub')

        self._left_cam_pub = self.create_publisher(
            Image,
            "telecom/zed/left/image_raw",
            10)
        
        self._right_cam_pub = self.create_publisher(
            Image,
            "telecom/zed/right/image_raw",
            10)

        self._timer = self.create_timer(1/fps, self._publish_frame)
        
        self._bridge = CvBridge()
        image_size = Resolution()

        self._camera = cv2.VideoCapture(0)
        self._camera.set(cv2.CAP_PROP_FPS, fps)
        self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width * 2)
        self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)

    def _publish_frame(self):
        success, frame = self._camera.read()
        
        if success:
            # split camera feed into left and right
            frame_left, frame_right = np.split(frame, 2, axis=1)

            self._left_cam_pub.publish(
                self._bridge.cv2_to_imgmsg(frame_left, "rgb8"))

            self._right_cam_pub.publish(
                self._bridge.cv2_to_imgmsg(frame_right, "rgb8"))

            self.get_logger().info('Publishing ZED frame')
        else:
            self.get_logger().info('Unsuccessful frame capture')


def main(args=None):
    """Crate and spin up node"""
    path = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                        '../../../../share/telecom/')
    
    with open(os.path.join(path, 'frame_data.json')) as frame_data_path:
        frame_data = json.load(frame_data_path)
        
        fps = frame_data['fps']

        rclpy.init(args=args)
        image_publisher = ZedPublisher(fps)
        rclpy.spin(image_publisher)

        image_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
