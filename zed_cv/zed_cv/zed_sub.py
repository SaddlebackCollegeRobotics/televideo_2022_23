import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import os
import sys
from datetime import datetime
import numpy as np


class CamSubscriber(Node):
    """Publishes compressed and uncompressed image feeds"""

    def __init__(self, gui: bool = True, verbose: bool = False):
        super().__init__('cam_sub')

        self.gui = gui
        self.verbose = verbose

        self._bridge = CvBridge()

        profile = qos_profile_sensor_data
        group = ReentrantCallbackGroup()
        self._subscription = self.create_subscription(CompressedImage,
                                                      'image_cv_compressed',
                                                      self.listener_callback,
                                                      qos_profile=profile,
                                                      callback_group=group)

    def listener_callback(self, data):
        """Called whenever we recieve a frame"""
        frame = self._bridge.compressed_imgmsg_to_cv2(data)

        if self.verbose:
            self.get_logger().info('Receiving ZED frame')
        if self.gui:
            cv2.imshow('Drive View', frame)
            cv2.waitKey(1)


def main(args=None):
    """Starts the node"""
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = CamSubscriber()
    cv2.namedWindow('Drive View', cv2.WINDOW_KEEPRATIO)

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    # Shutdown the ROS client library for Python
    rclpy.shutdown()                                         # Shutdown the rclpy library



if __name__ == '__main__':
    main()