"""ROS2 Node for publishing compressed and uncompressed images"""
import json
import os
import subprocess
import sys
from typing import NamedTuple
from collections import namedtuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


class Resolution(NamedTuple):
    """Default camera resolution."""
    width: int = 720
    height: int = 576


"""Holds camera video capture and publisher"""
Camera = namedtuple('Camera', ['cap', 'pub', 'id'])


class FpvPublisher(Node):
    """Publishes compressed and uncompressed image feeds"""

    def __init__(self, cam_dev_paths, fps: int = 24):
        super().__init__('fpv_pub')
        
        self._bridge = CvBridge()
        image_size = Resolution()
        self.cameras = []

        for id_, dev_path in enumerate(cam_dev_paths):
            cap = cv2.VideoCapture(dev_path)
            cap.set(cv2.CAP_PROP_FPS, fps)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)

            self.cameras.append(Camera(
                pub=self.create_publisher(Image, f'telecom/fpv/{id_}/image_raw', 10),
                cap=cap,
                id=id_
            ))

        self.create_timer(1/fps, self._publish_frames)
        

    def _publish_frames(self):
        for camera in self.cameras:
            success, frame = camera.cap.read()
            
            if success:
                camera.pub.publish(self._bridge.cv2_to_imgmsg(frame, "rgb8"))

                self.get_logger().info(f'Publishing Fpv Camera [{camera.id}] frame')
            else:
                self.get_logger().info('Unsuccessful frame capture')


def get_camera_dev_paths(self, path):
    """ Get motor controller device paths using serial IDs """
    getter_script = os.path.join(path, 'scripts/find_devpath.bash')
    
    device_list = subprocess.run(["\"" + getter_script + "\""], 
                                 stdout=subprocess.PIPE, 
                                 text=True, 
                                 shell=True, 
                                 executable='/bin/bash').stdout.splitlines()
    devpath_list = []

    # Add device paths to devpath list
    for device in device_list:
        splitStr = device.split(" - ")

        if 'MACROSIL_AV_TO_USB2.0' in splitStr[1]:
            devpath_list.append(splitStr[0])

    print(devpath_list)
    sys.exit()
    return devpath_list


def main(args=None):
    """Crate and spin up node"""
    path = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                        '../../../../share/telecom/')
    
    fpv_cameras = get_camera_dev_paths(path)

    with open(os.path.join(path, 'frame_data.json')) as frame_data_path:
        frame_data = json.load(frame_data_path)
        
        fps = frame_data['fps']

        rclpy.init(args=args)
        image_publisher = FpvPublisher(fpv_cameras, fps=fps)
        rclpy.spin(image_publisher)

        image_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
