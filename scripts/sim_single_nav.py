import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class LaserNode(Node):
    def __init__(self):
        super().__init__("LaserNode")
        self.laser = self.create_subscription(LaserScan,"/scan",self.laser_callback)

    def laser_callback(self,msg):
        print(msg)