#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Change '/lidar_topic' to your actual LiDAR topic
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # Change '/cmd_vel' to your actual velocity command topic
            10)
        self.min_distance = 0.2  # Minimum distance to obstacle

    def lidar_callback(self, msg):
        # Assuming LiDAR message structure is in standard format
        ranges = msg.ranges
        min_range = min(ranges)

        if min_range < self.min_distance:
            self.avoid_obstacle()
        else:
            self.move_forward()

    def avoid_obstacle(self):
        # Stop and turn
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.5  # Adjust angular velocity as needed
        self.publisher.publish(twist_msg)

    def move_forward(self):
        # Move forward
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Adjust linear velocity as needed
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
