#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Testing fire tracking")
        self.camera_sub = self.create_subscription(Image, "/image_raw", self.camera_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.bridge = CvBridge()
        self.fire_cascade = cv2.CascadeClassifier('/home/robot/project/src/my_robot/fire_detection.xml')
        self.image_width = None
        self.image_height = None
        self.margin_of_error = 20  # Margin of error for centering the bounding box
        self.threshold_area = 5000  # Threshold area for stopping the robot
        self.linear_speed = 0.2  # Linear velocity
        self.angular_speed = 0.3  # Angular velocity for rotation
        self.stop_distance = 0.4  # Stop distance from the fire (40 cm)

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        fire = self.fire_cascade.detectMultiScale(img, 1.2, 5)
        self.image_width = img.shape[1]
        self.image_height = img.shape[0]
        
        for (x, y, width, height) in fire:
            if abs(x + width / 2 - self.image_width / 2) < self.margin_of_error \
                and abs(y + height / 2 - self.image_height / 2) < self.margin_of_error:
                if width * height > self.threshold_area:
                    self.stop_robot()
                    return
            elif x + width / 2 < self.image_width / 2:
                # Fire detected on the left side, rotate left
                self.rotate_left()
            else:
                # Fire detected on the right side, rotate right
                self.rotate_right()

        # If no fire detected or not in desired location, move forward
        self.move_forward()

    def move_forward(self):
        move = Twist()
        move.linear.x = self.linear_speed
        move.angular.z = 0.0
        self.cmd_vel_pub.publish(move)

    def rotate_left(self):
        move = Twist()
        move.linear.x = 0.0
        move.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(move)

    def rotate_right(self):
        move = Twist()
        move.linear.x = 0.0
        move.angular.z = -self.angular_speed
        self.cmd_vel_pub.publish(move)

    def stop_robot(self):
        move = Twist()
        move.linear.x = 0.0
        move.angular.z = 0.0
        self.cmd_vel_pub.publish(move)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
