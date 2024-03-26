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
        self.get_logger().info("Testing fire tracking with stopping")
        self.camera_sub = self.create_subscription(Image, "/image_raw", self.camera_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.bridge = CvBridge()
        self.fire_cascade = cv2.CascadeClassifier('/home/robot/project/src/my_robot/fire_detection.xml')
        self.target_x = None
        self.image_width = None
        self.k_p = 0.001  # Proportional control gain
        self.linear_speed = 0.2  # Linear velocity for approach
        self.approach_distance = 0.5  # Minimum distance for stopping (in meters)
        self.flame_pin = 4  # GPIO pin connected to flame sensor (replace with actual pin)
        self.GPIO_setup()
        self.timer = self.create_timer(0.1, self.send_cmd_vel)

    def GPIO_setup(self):
        import RPi.GPIO as GPIO  # Import GPIO library here
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.flame_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def send_cmd_vel(self):
        if self.target_x is None or self.image_width is None:
            # Stop the robot if no fire detected
            self.stop_robot()
            return

        error = self.target_x - self.image_width / 2  # Calculate error (difference from center)
        angular_vel = -self.k_p * error  # Proportional control: angular velocity proportional to error

        # Check flame sensor for stopping near fire
        if self.is_fire_near():
            self.stop_robot()
        else:
            move = Twist()
            move.linear.x = self.linear_speed  # Set linear velocity for approach
            move.angular.z = angular_vel
            self.cmd_vel_pub.publish(move)

    def stop_robot(self):
        # Stop the robot's motion
        move = Twist()
        self.cmd_vel_pub.publish(move)

    def is_fire_near(self):
        import RPi.GPIO as GPIO  # Import GPIO library here
        flame_detected = GPIO.input(self.flame_pin)
        return flame_detected == 0  # Check for LOW signal indicating fire

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        fire = self.fire_cascade.detectMultiScale(img, 1.2, 5)
        if len(fire) > 0:  # If fire detected
            (x, y, w, h) = fire[0]  # Use the first detected fire
            self.target_x = x + w / 2  # Update target x-coordinate (center of the fire)
        else:
            self.target_x = None  # No fire detected, reset target x-coordinate
        self.image_width = img.shape[1]  # Get image width

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
