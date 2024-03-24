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
        self.model = YOLO('/home/bsusheelkumar/final_year_project/src/my_robot/trained.engine')
        self.camera_sub = self.create_subscription(Image, "image_raw", self.camera_callback, 10)
        self.img_pub = self.create_publisher(Image, "camera_output", 1)
        self.class_names = ['fire']
        self.bridge = CvBridge()
        self.fire_detected = False  # Track if fire is detected

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img, stream=True)

        # Reset fire detection flag
        self.fire_detected = False

        for r in results:
            boxes = r.boxes
            for box in boxes:
                x_min, y_min, x_max, y_max = box.xyxy[0]
                x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
                cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (255, 0, 255), 3)
                conf = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])

                if cls == 0:
                    cv2.putText(img, f'{self.class_names[cls]} {conf}', (x_min, y_min - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                    self.fire_detected = True  # Set flag when fire is detected

        if self.fire_detected:
            GPIO.output(18, 0)  # Turn on relay
        else:
            GPIO.output(18, 1)  # Turn off relay

        img_to_pub = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.img_pub.publish(img_to_pub)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == "__main__":
    main()
