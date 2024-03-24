#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Testing fire tracking")
        self.camera_sub = self.create_subscription(Image, "/image_raw", self.camera_callback, 10)
        self.bridge = CvBridge()
        self.fire_cascade = cv2.CascadeClassifier('/home/robot/project/src/my_robot/fire_detection.xml')

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        fire = self.fire_cascade.detectMultiScale(img, 1.2, 5)
        for (x, y, w, h) in fire:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            print(f"Bounding box: x={x}, y={y}, width={w}, height={h}")
        # cv2.imshow('Fire Detection', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
