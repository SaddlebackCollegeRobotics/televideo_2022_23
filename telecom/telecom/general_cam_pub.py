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
    width: int
    height: int


class CameraPublisher(Node):

    def __init__(self, target_resolution: Resolution, fps: int = 24):

        super().__init__('cam_pub')

        self.camera_pub = self.create_publisher(
            Image,
            "/image_raw",
            10)

        self._timer = None
        
        self._bridge = CvBridge()
        self.image_cap_size = Resolution(720, 576) # FPV cam resolution
        self.target_resolution = target_resolution

        while (True):
            index = input("Camera index: ")
            self.create_camera(self.image_cap_size, fps, index)


    def create_camera(self, image_cap_size, fps, index):

        self._camera = cv2.VideoCapture(index)

        self._camera.set(cv2.CAP_PROP_FPS, fps)
        self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, image_cap_size.width)
        self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, image_cap_size.height)

        del self._timer
        self._timer = self.create_timer(1/fps, self._publish_frame)


    def _publish_frame(self):
        
        success, frame = self._camera.read()
        
        if success:
            frame = cv2.resize(frame, self.target_resolution)
            self.camera_pub.publish(self._bridge.cv2_to_imgmsg(frame, "rgb8"))
        else:
            self.get_logger().info('Unsuccessful frame capture')


def main(args=None):

    config_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                        '../../../../share/telecom/')
    
    with open(os.path.join(config_path, 'frame_data.json')) as frame_data_path:
        frame_data = json.load(frame_data_path)

        rclpy.init(args=args)
        camera_publisher = CameraPublisher(target_resolution=Resolution(frame_data['height'], frame_data['width']), 
                                           fps=frame_data['fps'])
        rclpy.spin(camera_publisher)

        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
