#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Testing fire tracking")
        self.camera_sub = self.create_subscription(Image, "/image_raw", self.camera_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.bridge = CvBridge()
        self.fire_cascade = cv2.CascadeClassifier('/home/bsusheelkumar/final_year_project/src/my_robot/fire_detection.xml')
        self.target_x = None
        self.image_width = None
        self.k_p = 0.001  # Proportional control gain
        self.linear_speed = 0.2  # Linear velocity
        self.angular_speed = 0.5  # Angular velocity for rotation
        self.timer = self.create_timer(0.1, self.send_cmd_vel)
        self.fire_detected = False

    def send_cmd_vel(self):
        if self.fire_detected:
            # Stop the robot near the fire and face the fire
            self.stop_robot()
            return

        if self.target_x is None or self.image_width is None:
            # Rotate the robot if no fire detected
            self.rotate_robot()
            return

        error = self.target_x - self.image_width / 2  # Calculate error (difference from center)
        angular_vel = -self.k_p * error  # Proportional control: angular velocity proportional to error

        move = Twist()
        move.linear.x = self.linear_speed  # Set linear velocity
        move.angular.z = angular_vel
        self.cmd_vel_pub.publish(move)

    def stop_robot(self):
        # Stop the robot's motion
        move = Twist()
        self.cmd_vel_pub.publish(move)

    def rotate_robot(self):
        # Rotate the robot to search for fire
        move = Twist()
        move.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(move)

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        fire = self.fire_cascade.detectMultiScale(img, 1.2, 5)
        if len(fire) > 0:  # If fire detected
            (x, y, w, h) = fire[0]  # Use the first detected fire
            self.target_x = x + w / 2  # Update target x-coordinate (center of the fire)
            self.fire_detected = True
        else:
            self.target_x = None  # No fire detected, reset target x-coordinate
            self.fire_detected = False
        self.image_width = img.shape[1]  # Get image width

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
