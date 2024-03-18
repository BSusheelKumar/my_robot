#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class MyNode(Node):

    def  __init__(self):
        super().__init__("my_node")
        self.get_logger().info("testing cv2")
        self.camera_sub = self.create_subscription(Image,"/camera/image_raw",self.camera_callback,10)
        self.img_pub = self.create_publisher(Image,"camera_output",1)
        self.bridge = CvBridge()
    def camera_callback(self,data):
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        cv2.imshow("output",img)
        img_to_pub = self.bridge.cv2_to_imgmsg(img,"bgr8")
        self.img_pub.publish(img_to_pub)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()