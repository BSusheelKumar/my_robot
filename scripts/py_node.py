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
        self.model = YOLO('yolov8n.pt')
        self.camera_sub = self.create_subscription(Image,"image_raw",self.camera_callback,10)
        self.img_pub = self.create_publisher(Image,"camera_output",1)
        self.cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel",10)
        self.timer = self.create_timer(0.5,self.send_vel_cmd)
        self.get_logger().info("Navigating to person")
        self.bridge = CvBridge()
        self.class_names = [
    "person", "bicycle", "car", "motorcycle", "airplane",
    "bus", "train", "truck", "boat", "traffic light", "fire hydrant",
    "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush"
]
    def send_vel_cmd(self,linear_val=0.0,angular_val=0.0):
        msg = Twist()
        msg.linear.x=linear_val
        msg.angular.z=angular_val
        self.cmd_vel_pub.publish(msg)
 
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
                    # if (abs(x_deviation) < tolerance):
                        
                    #     if (y_max>0.9):
                    #         self.send_vel_cmd(0.0,0.0)
                    #     else:
                    #         self.send_vel_cmd(1.0,0.0)

                    # else:
                    #     if (x_deviation>=tolerance):
                    #         time.sleep(1)
                    #         self.send_vel_cmd(0.0,1.0)

                    #     if (x_deviation<= -1 +tolerance):
                    #         time.sleep(1)
                    #         self.send_vel_cmd(0.0,-1.0)

        img_to_pub = self.bridge.cv2_to_imgmsg(img,"bgr8")
        self.img_pub.publish(img_to_pub)
        cv2.imshow("output",img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()