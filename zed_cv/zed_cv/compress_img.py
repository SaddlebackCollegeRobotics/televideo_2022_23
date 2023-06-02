from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage, Image


class CompressImage(Node):

    def __init__(self):
        super().__init__('compress_img')

        self.create_subscription(Image, 
                                 '/image_raw', 
                                 self._publish_compressed_img,  
                                 qos_profile=qos_profile_sensor_data)
        self._camera_pub = self.create_publisher(CompressedImage, 
                                                 '/image_compressed', 
                                                 10)
        self._bridge = CvBridge()

    def _publish_compressed_img(self, msg):
        img = self._bridge.imgmsg_to_cv2(msg.data)
        compressed = self._bridge.cv2_to_compressed_imgmsg(img)
        self._camera_pub.publish(compressed)


def main(args=None):
    rclpy.init(args=args)
    compress_image = CompressImage()
    rclpy.spin(compress_image)

    compress_image.destroy_node()
    rclpy.shutdown()
