"""ROS2 Node for publishing compressed and uncompressed images"""
from typing import NamedTuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class Resolution(NamedTuple):
    """Desired camera resolution."""
    # width: int = 2208
    # height: int = 1242
    width: int = 1280
    height: int = 720


class ZedPublisher(Node):
    """Publishes compressed and uncompressed image feeds"""

    def __init__(self, compressed_size: tuple, fps: int = 30):
        super().__init__('zed_pub')

        self.compressed_size = compressed_size

        # publish raw (2K), resized (10), and compressed images
        self._raw_pub = self.create_publisher(
            Image,
            "telecom/image_raw",
            10)

        self._resized_pub = self.create_publisher(
            Image,
            '/image_raw',
            10)

        self._compressed_pub = self.create_publisher(
            CompressedImage,
            '/telecom/image_compressed',
            qos_profile=qos_profile_sensor_data)

        self._timer = self.create_timer(1/15, self._timer_callback)
        self._camera = cv2.VideoCapture(0)
        # self._camera.set(cv2.CAP_PROP_BUFFERSIZE, 10)
        self._camera.set(cv2.CAP_PROP_FPS, 15)
        self._bridge = CvBridge()
        image_size = Resolution()
        self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width * 2)
        self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)

    def _timer_callback(self):
        success, frame = self._camera.read()
        
        if success:
            # only take left camera feed
            frame = np.split(frame, 2, axis=1)[0]

            # publish uncompressed frame
            # self._raw_pub.publish(self._bridge.cv2_to_imgmsg(frame, "rgb8"))

            frame = cv2.resize(frame,
                            #    self.compressed_size,
                                (1024, 576),
                               interpolation=cv2.INTER_AREA)

            # publish resized frame
            self._resized_pub.publish(self._bridge.cv2_to_imgmsg(frame, "rgb8"))

            # publish compressed frame
            self._compressed_pub.publish(
                self._bridge.cv2_to_compressed_imgmsg(frame))

            self.get_logger().info('Publishing ZED frame')
        else:
            self.get_logger().info('Unsuccessful frame capture')


def main(args=None):
    """Crate and spin up node"""
    compressed_size = (1920, 1080)

    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    image_publisher = ZedPublisher(compressed_size)
    # Spin the node so the callback function is called.
    rclpy.spin(image_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()
    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
