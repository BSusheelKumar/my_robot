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

    def  __init__(self):
        super().__init__("my_node")
        self.get_logger().info("testing cv2")
        self.model = YOLO('final.pt')
        self.camera_sub = self.create_subscription(Image,"image_raw",self.camera_callback,10)
        self.img_pub = self.create_publisher(Image,"camera_output",1)
        # self.cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel",10)
        # self.timer = self.create_timer(0.5,self.send_vel_cmd)
        self.class_names = ['fire']

    def camera_callback(self,data):
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        results = self.model(img,stream=True)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x_min, y_min, x_max, y_max= box.xyxy[0]
                x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
                print(x_min, y_min, x_max, y_max)
                cv2.rectangle(img,(x_min,y_min),(x_max,y_max),(255,0,255),3)

                conf = math.ceil((box.conf[0]*100))  /100
                tolerance=0.1
                x_deviation=0
                y_max=0

                cls = int(box.cls[0])

                if cls == 0:
                    cv2.putText(img, f'{self.class_names[cls]} {conf}', (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                    x_diff = x_max - x_min
                    y_diff = y_max - y_min

                    obj_x_centre = x_min+(x_diff/2)
                    obj_x_centre = round(obj_x_centre,3)

                    x_deviation = round(0.5-obj_x_centre,3)
                    y_max = round(y_max,3)
                    print("{",x_deviation,y_max,"}")
        img_to_pub = self.bridge.cv2_to_imgmsg(img,"bgr8")
        self.img_pub.publish(img_to_pub)
        # cv2.imshow("output",img)
        cv2.waitKey(1)
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()