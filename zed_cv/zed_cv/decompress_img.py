from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image


class DecompressImage(Node):

    def __init__(self):
        self.create_subscription(Image, '/image_compressed', self._publish_uncompressed_img, 10)
        self._camera_pub = self.create_publisher(CompressedImage, '/image_uncompressed', 10)
        self._bridge = CvBridge()

    def _publish_uncompressed_img(self, msg):
        compressed = self._bridge.compressed_imgmsg_to_cv2(msg.data)
        img = self._bridge.cv2_to_imgmsg(img)
        self._camera_pub.publish(img)


def main(args=None):
    rclpy.init(args=args)
    decompress_image = DecompressImage()
    rclpy.spin(decompress_image)

    decompress_image.destroy_node()
    rclpy.shutdown()