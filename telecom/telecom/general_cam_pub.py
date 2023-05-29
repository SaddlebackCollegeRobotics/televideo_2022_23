"""ROS2 Node for publishing compressed and uncompressed images"""
import json
import os
from typing import NamedTuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

import cv2
from cv_bridge import CvBridge
import numpy as np
import threading


class Resolution(NamedTuple):
    width: int
    height: int


class CameraPublisher(Node):

    def __init__(self, target_resolution: Resolution, fps: int = 24):

        super().__init__('cam_pub2')

        self.target_resolution = target_resolution
        self.fps = fps

        self.camera_pub = self.create_publisher(
            Image,
            "/image_raw",
            10)
        
        self.camera_controls_sub = self.create_subscription(
            Int32,
            "/camera_controls",
            self.camera_controls_callback,
            10)
            

        self._camera = None
        self.camera_index = 0
        
        self._bridge = CvBridge()
        self.image_cap_size = Resolution(720, 576) # FPV cam resolution

        self._timer = self.create_timer(1/fps, self._publish_frame)
        

    def camera_controls_callback(self, msg):

        print("Camera index set to: " + str(msg.data))
        self.camera_index = msg.data
        self._camera = self.create_camera(msg.data)


    def create_camera(self, index):

        camera = cv2.VideoCapture(index)

        camera.set(cv2.CAP_PROP_FPS, self.fps)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_cap_size.width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_cap_size.height)

        return camera


    def _publish_frame(self):
        
        if self._camera is None:
            return

        success, frame = self._camera.read()
        
        if success:
            
            self.get_logger().info(f'Publishing Camera [{self.camera_index}] frame')

            frame = cv2.resize(frame, self.target_resolution)
            self.camera_pub.publish(self._bridge.cv2_to_imgmsg(frame, "rgb8"))
        else:
            self.get_logger().info(f'Failed to read Camera [{self.camera_index}] frame')



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
