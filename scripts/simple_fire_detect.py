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
        self.fire_cascade = cv2.CascadeClassifier('/home/bsusheelkumar/final_year_project/src/my_robot/fire_detection.xml')
        self.start_time = time.time()
        self.frame_count = 0
        self.cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel",10)
        self.timer = self.create_timer(0.1,self.send_cmd_vel)

    def send_cmd_vel(self,linear_val=0.0,angular_val=0.0):
        move = Twist()
        move.linear.x = linear_val
        move.angular.z = angular_val
        self.cmd_vel_pub.publish(move)


    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        fire = self.fire_cascade.detectMultiScale(img, 1.2, 5)
        for (x, y, w, h) in fire:
            frame_width_middle = img.shape[1] / 2
            if x < frame_width_middle - 50:
                print("Fire detected on the left side")
                self.send_cmd_vel(0.0,1.0)
            elif x > frame_width_middle + 50:
                print("Fire detected on the right side")
                self.send_cmd_vel(0.0,-1.0)
            else:
                print("Fire detected in the middle")
                self.send_cmd_vel(0.0,0.0)
                
            cv2.rectangle(img, (x - 20, y - 20), (x + w + 20, y + h + 20), (255, 0, 0), 2)
            roi_gray = gray[y:y + h, x:x + w]
            roi_color = img[y:y + h, x:x + w]

        self.frame_count += 1
        if time.time() - self.start_time >= 1:
            fps = self.frame_count / (time.time() - self.start_time)
            print(f"FPS: {fps:.2f}")
            self.frame_count = 0
            self.start_time = time.time()
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
