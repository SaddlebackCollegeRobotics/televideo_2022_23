"""ROS2 Node for subscribing to camera feed and displaying images"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


class CamSubscriber(Node):
    """Publishes compressed and uncompressed image feeds"""

    def __init__(self, gui: bool = True):
        super().__init__('cam_sub')

        self._gui = gui
        self._bridge = CvBridge()

        profile = qos_profile_sensor_data
        group = ReentrantCallbackGroup()
        
        self.create_subscription(
            Image,
            '/image_uncompressed',
            self.listener_callback,
            qos_profile=profile,
            callback_group=group)

    def listener_callback(self, data):
        """Called whenever we recieve a frame"""
        frame = self._bridge.imgmsg_to_cv2(data)

        if self._gui:
            cv2.imshow('Drive View', frame)
            cv2.waitKey(1)
        else:
            self.get_logger().info('Receiving Cameron frame 📷')


def main(args=None):
    """Starts the node"""
    rclpy.init(args=args)

    image_subscriber = CamSubscriber()
    cv2.namedWindow('Drive View', cv2.WINDOW_KEEPRATIO)
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown() 


if __name__ == '__main__':
    main()
