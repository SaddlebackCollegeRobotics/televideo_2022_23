"""ROS2 Node for publishing compressed and uncompressed images"""
import os
from typing import NamedTuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np

class Resolution(NamedTuple):
    """Desired camera resolution."""
    width: int = 1920
    height: int = 1080

class ImageStitcher(Node):
    """Publishes compressed and uncompressed image feeds"""

    def __init__(self, resolution, fps: int = 15):
        super().__init__('image_stitcher')

        self.resolution = resolution
        self.channels = [
            'telecom/zed/left/image_raw', 
            'telecom/zed/left/image_raw',
            'telecom/cam1/image_raw',
            'telecom/cam2/image_raw',
            'telecom/cam3/image_raw',
            'telecom/cam4/image_raw',
        ]
        self.cam_subscriptions = dict()
        self.images = dict()

        self._profile = qos_profile_sensor_data
        self._bridge = CvBridge()

        self._stitched_frame_publisher = self.create_publisher(
            Image,
            "/image_raw",
            10)

        self.create_timer(1/fps, self._publish_stitched_frame)

        timer_period = 0.1
        self.create_timer(timer_period, self._subscribe_to_cams_cli)
        
    def _subscribe_to_cams_cli(self):
        # Select a channel
        channel_num = 0

        while True:
            print("Please select a channel from the list below:")
            for i, channel in enumerate(self.channels):
                print(f'[{i}] {channel}')

            channel_num = int(input("> "))
            
            if 0 <= channel_num < len(self.channels):
                break

        channel = self.channels[channel_num]

        # Select add or remove
        while True: 
            print("[0] to add channel") 
            print("[1] to remove channel")
            print("[2] to go back")
            option = int(input("> "))

            # Add a new channel subscriber
            if option == 0 and channel not in self.cam_subscriptions:
                # create callback function to convert image
                def callback(data):
                    self.images[channel] = self._bridge.imgmsg_to_cv2(data)

                # add subscription to dictionary of camera subscriptions
                self.cam_subscriptions[channel] = self.create_subscription(
                    Image, channel, callback, qos_profile=self._profile)
               
                print(f"Channel '/{channel}' subscribed\n")
                break

            # Remove a channel subscriber
            if option == 1:
                unsubscribed = False
                if channel in self.cam_subscriptions:
                    del self.cam_subscriptions[channel]
                    unsubscribed = True
                if channel in self.images:
                    del self.images[channel]

                if unsubscribed:
                    print(f"Channel '/{channel}' unsubscribed\n")
                break

            if option == 2:
                print()
                break

    def _publish_stitched_frame(self):
        
        if len(self.images) == 1:
            img_size = self.resolution
        else:
            img_size = (self.resolution.width/2, self.resolution.height/2)

        images = [cv2.resize(img, img_size) for _, img in self.images]

        stitched_image = self.stitch_images(images)

        self._stitched_frame_publisher.publish(
            self._bridge.cv2_to_imgmsg(stitched_image, 'rgb8'))
    
    def stitch_images(self, images):
        # note: axis=0 - vertical stitch, axis=1 - horizontal stitch
        # np.pad(arr, ((top,bottom), (left,right)), 'constant')
        match len(images):
            case 0:
                return np.zeros((*self.resolution, 3), dtype=np.uint8)
            case 1: 
                return images[0]
            case 2:
                return np.concatenate(images[0], images[1], axis=1)
            case 3:
                top = np.concatenate((images[0], images[1]), axis=1)
                horiz_padding = (images[2].shape[0]/2, images[2].shape[0]/2)
                bottom = np.pad(images[2], ((0, 0), horiz_padding))
                return np.concatenate((top, bottom), axis=0)
            case 4:
                top = np.concatenate((images[0], images[1]), axis=1)
                bottom = np.concatenate((images[2], images[3]), axis=1)
                return np.concatenate((top, bottom), axis=0)


def main(args=None):
    """Crate and spin up node"""
    path = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                        '../../../../share/drive')

    with open(os.path.join(path, 'frame_data.json')) as frame_data:
        resolution = Resolution(frame_data['width'], frame_data['height'])
        fps = frame_data['fps']
        rclpy.init(args=args)

        image_publisher = ImageStitcher(resolution, fps=fps)
        rclpy.spin(image_publisher)

        image_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
