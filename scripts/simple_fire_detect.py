#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import math
from geometry_msgs.msg import Twist
import time

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("testing cv2")
        self.camera_sub = self.create_subscription(Image, "image_raw", self.camera_callback, 10)
        self.img_pub = self.create_publisher(Image, "camera_output", 1)
        self.class_names = ['fire']
        self.bridge = CvBridge()
        self.fire_detected = False  # Track if fire is detected
        self.fire_cascade = cv2.CascadeClassifier('/home/robot/project/src/my_robot/fire_detection.xml')

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        fire = self.fire_cascade.detectMultiScale(img, 1.2, 5)
        for (x,y,w,h) in fire:
            cv2.rectangle(frame,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = frame[y:y+h, x:x+w]
            print("fire is detected")
        
        img_to_pub = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.img_pub.publish(img_to_pub)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    # GPIO.cleanup()

if __name__ == "__main__":
    main()
