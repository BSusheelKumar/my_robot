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
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

# Set pin 23 (modify if using a different pin) as output
GPIO.setup(23, GPIO.OUT)

class MyNode(Node):

    def  __init__(self):
        super().__init__("my_node")
        self.get_logger().info("testing cv2")
        self.model = YOLO('/home/robot/project/src/my_robot/trained.pt')
        self.camera_sub = self.create_subscription(Image,"image_raw",self.camera_callback,10)
        self.img_pub = self.create_publisher(Image,"camera_output",1)
        # self.cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel",10)
        # self.timer = self.create_timer(0.5,self.send_vel_cmd)
        self.class_names = ['fire']
        self.bridge = CvBridge()
        self.if_fire_detected = False
        

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


                if cls == 0:  # Fire is detected
                    
                    cv2.putText(img, "Fire Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)  # Add this line
                    self.if_fire_detected = True
                    GPIO.output(23, GPIO.HIGH)
                else:
                    self.if_fire_detected = False
        if not self.if_fire_detected:
            GPIO.output(23, GPIO.LOW)
        img_to_pub = self.bridge.cv2_to_imgmsg(img,"bgr8")
        self.img_pub.publish(img_to_pub)
        # cv2.imshow("output",img)
        cv2.waitKey(1)
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == "__main__":
    main()