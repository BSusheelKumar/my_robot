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
        self.img_pub = self.create_publisher(Image,"/output_image",1)
        self.bridge = CvBridge()
        self.fire_cascade = cv2.CascadeClassifier('/home/bsusheelkumar/final_year_project/src/my_robot/fire_detection.xml')
        self.target_x = None
        self.image_width = None
        self.k_p = 0.001  # Proportional control gain
        self.linear_speed = 0.2  # Linear velocity
        self.stop_distance = 0.5  # Distance to stop when fire is detected (in meters, adjust as needed)
        self.timer = self.create_timer(0.1, self.send_cmd_vel)

    def send_cmd_vel(self):
        if self.target_x is None or self.image_width is None:
            # Stop the robot if no fire detected
            self.stop_robot()
            return

        error = self.target_x - self.image_width / 2  # Calculate error (difference from center)
        angular_vel = self.k_p * error  # Proportional control: angular velocity proportional to error

        # Check if fire is within stopping distance based on bounding boxes
        if self.is_fire_close_enough():
            self.stop_robot()
            return

        move = Twist()
        move.linear.x = self.linear_speed  # Set linear velocity
        move.angular.z = angular_vel
        self.cmd_vel_pub.publish(move)

    def stop_robot(self):
        # Stop the robot's motion
        move = Twist()
        self.cmd_vel_pub.publish(move)

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Use the provided bounding boxes (replace if fire detection gives different data)
        self.bounding_boxes = [
            (241, 236, 121, 121),
            (216, 215, 154, 154),
            (220, 219, 154, 154),
            # ... other bounding boxes from your measurements
        ]

        if len(self.bounding_boxes) > 0:  # If any bounding boxes present
            self.target_x = (self.bounding_boxes[0][0] + self.bounding_boxes[0][0] + self.bounding_boxes[0][2]) // 2
            # Assuming you want to stop based on the center of the first bounding box
        else:
            self.target_x = None  # No fire detected, reset target x-coordinate

        self.image_width = img.shape[1]  # Get image width
        img_to_pub = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.img_pub.publish(img_to_pub)

    def is_fire_close_enough(self):
        # Check if any bounding box center is within the stopping distance
        if self.target_x is None:
            return False

        for (x, y, w, h) in self.bounding_boxes:
            fire_center_x = x + w // 2  # Calculate center of the bounding box
            distance_to_fire = abs(fire_center_x - self.image_width / 2)  # Approximate distance based on image center

            if distance_to_fire <= self.stop_distance * self.image_width:  # Check if within stopping distance
                return True

        return False  # No fire close enough

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

