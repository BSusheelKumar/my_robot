#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Recieve(Node):
    def __init__(self):
        super().__init__("recieve")
        self.get_logger().info("Started rec")
        self.recieve = self.create_subscription(String,"/search_fire",self.read_callback, 10)

    def read_callback(self,msg):
            self.get_logger().info(f"recevied{msg.data}")


def main(args=None):
    rclpy.init(args=args)
    re_node = Recieve()
    rclpy.spin(re_node)
    re_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

