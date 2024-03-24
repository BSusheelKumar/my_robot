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
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

# Set pin 23 (modify if using a different pin) as output
GPIO.setup(18, GPIO.OUT)

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("testing cv2")
        self.camera_sub = self.create_subscription(Image, "image_raw", self.camera_callback, 10)
        self.img_pub = self.create_publisher(Image, "camera_output", 1)
        self.class_names = ['fire']
        self.bridge = CvBridge()
        self.fire_detected = False  # Track if fire is detected
    
    def camera_callback(self,data):
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        cap = cv2.VideoCapture(img)
        while cap.isOpened():
            ret,frame = cap.read()
            if not ret:
                break
            fire_detected, fire_contours = self.detect_fire(frame)

            if fire_detected:
                print("fire Detected")
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cap.release()
                break
        img_to_pub = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.img_pub.publish(img_to_pub)
    def detect_fire(self,frame):
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for fire color in HSV
        lower_bound = (0, 100, 100)
        upper_bound = (10, 255, 255)

        # Create a mask to threshold the frame for the fire color
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Apply morphology operations to remove noise and smooth the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check if any contours (fire) are detected
        if contours:
            return True, contours
        else:
            return False, []
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == "__main__":
    main()
