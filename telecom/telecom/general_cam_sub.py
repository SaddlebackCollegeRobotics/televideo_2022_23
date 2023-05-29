"""ROS2 Node for subscribing to camera feed and displaying images"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

import cv2
from cv_bridge import CvBridge
import threading

class CamSubscriber(Node):
    """Publishes compressed and uncompressed image feeds"""

    def __init__(self, gui: bool = True):
        super().__init__('cam_sub2')

        self._gui = gui
        self._bridge = CvBridge()

        self.create_subscription(
            Image,
            # '/image_uncompressed',
            '/image_raw',
            self.listener_callback,
            10)
        
        self.controls_pub = self.create_publisher(
            Int32,
            "/camera_controls",
            10)
        
        self.cool_thread = threading.Thread(target=self.input_loop)
        self.cool_thread.start()


    def input_loop(self):
        while True:
            index = input("Enter camera index: ")
            int_wrapper = Int32()
            int_wrapper.data = int(index)
            self.controls_pub.publish(int_wrapper) 


    def listener_callback(self, data):
        """Called whenever we recieve a frame"""
        frame = self._bridge.imgmsg_to_cv2(data)

        if self._gui:
            cv2.imshow('Drive View', frame)
            cv2.waitKey(1)


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
