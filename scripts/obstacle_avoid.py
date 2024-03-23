#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Obstacle_node(Node):
    def __init__(self):
        super().__init__("obstacle_avoiding")
        self.subscription = self.create_subscription(LaserScan,"/scan",self.scan_callback,10)
        self.publisher = self.create_publisher(Twist,"/cmd_vel",10)
        self.timer = self.create_timer(0.1,self.publish_velocities)
        self.min_distance_threshold = 0.2
        self.scan_data = []
        self.g_range_ahead = 1

    def scan_callback(self,msg):
        
        self.scan_data = msg.ranges
        self.g_range_ahead = min(msg.ranges[:60] + msg.ranges[-60:])


    def publish_velocities(self):
        twist = Twist()
        driving_forward = True
        if driving_forward:
            if self.g_range_ahead < 1.0:
                driving_forward = False
            else:
                driving_forward = True
        if driving_forward:
            twist.linear.x = 0.1
        else:
            twist.angular.z = 1.0

        self.publisher.publish(twist)






def main(args=None):
    rclpy.init(args=args)
    node = Obstacle_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()