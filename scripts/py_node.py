#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Send(Node):
    def __init__(self):
        super().__init__("sending_node")
        self.get_logger().info("Starting send")
        self.send = self.create_publisher(String, "/sending", 10)
        self.timer = self.create_timer(0.1, self.send_callback)
        self.msgs = ["hello", "hi", "yo","start_searching"]
        self.index = 0


    def send_callback(self):
        msg = String()
        msg.data = self.msgs[self.index]
        self.get_logger().info("publishing: %s" % msg.data)
        self.send.publish(msg)
        self.index = (self.index + 1) % len(self.msgs)

def main(args=None):
    rclpy.init(args=args)
    node = Send()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
