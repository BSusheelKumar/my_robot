#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Testing fire tracking with stopping")
        self.camera_sub = self.create_subscription(Image, "/image_raw", self.camera_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.image_publisher = self.create_publisher(Image,"/fire_detection_image",10)
        self.done_signal = self.create_publisher(String,"/done_with_fire",10)
        self.bridge = CvBridge()
        self.fire_cascade = cv2.CascadeClassifier('/home/robot/final_year_project/src/my_robot/fire_detection.xml')
        self.target_x = None
        self.image_width = None
        self.k_p = 0.001  # Proportional control gain
        self.linear_speed = 0.2  # Linear velocity for approach
        self.approach_distance = 0.5  # Minimum distance for stopping (in meters)
        self.flame_pin = 4  # GPIO pin connected to flame sensor (replace with actual pin)
        self.GPIO_setup()
        self.timer = self.create_timer(0.1, self.send_cmd_vel)
        self.relay_pin = 27  # GPIO pin connected to relay (replace with actual pin)
        self.GPIO_setup_relay()
        self.recieve_signal = self.create_subscription(String,"/search_fire",self.read_callback,10)
        self.reached_goal = False
        self.fire_detected = False


    def read_callback(self,msg):
        if msg.data == "reached_goal":
            self.get_logger().info("Reached Goal Starting Image Processing")
            self.reached_goal = True
            
        else:
            self.get_logger().info("Goal Not Reached Yet")
            self.reached_goal = False
    

    def GPIO_setup(self):
        import RPi.GPIO as GPIO  # Import GPIO library here
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.flame_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


    def GPIO_setup_relay(self):
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pin, GPIO.OUT)
        GPIO.output(self.relay_pin, GPIO.HIGH) 

    def send_cmd_vel(self):
        if not self.reached_goal:
            return  
        if self.target_x is None or self.image_width is None:
            print("no fire detected")
            # Stop the robot if no fire detected
            self.stop_robot()
            return

        error = self.target_x - self.image_width / 2  
        angular_vel = -self.k_p * error 

        if self.is_fire_near():
            self.stop_robot()
            msg = String()
            if self.fire_detected ==False:
                msg.data = "done"
                self.done_signal.publish(msg)
            else:
                msg.data = "not done"
                self.done_signal.publish(msg)

        else:
            move = Twist()
            move.linear.x = self.linear_speed 
            move.angular.z = angular_vel
            self.cmd_vel_pub.publish(move)

    def stop_robot(self):
        # Stop the robot's motion
        move = Twist()
        self.cmd_vel_pub.publish(move)

    def start_rotating(self):
        move = Twist()
        move.angular.z = 1.0
        self.cmd_vel_pub.publish(move)

    def is_fire_near(self):
        import RPi.GPIO as GPIO
        flame_detected = GPIO.input(self.flame_pin)
        if flame_detected == 0:  # Check for LOW signal indicating fire
            GPIO.output(self.relay_pin, GPIO.LOW)  # Turn on the relay if fire detected
            return True
        else:
            GPIO.output(self.relay_pin, GPIO.HIGH)
            # GPIO.cleanup()  # Turn off the relay if no fire detected
            return False

    def camera_callback(self, data):
         if self.reached_goal:
            # self.start_rotating()
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            fire = self.fire_cascade.detectMultiScale(img, 1.2, 5)
            print("searching for fire")
            if len(fire) > 0:
                self.fire_detected = True
                # self.stop_robot()  # If fire detected
                (x, y, w, h) = fire[0]  # Use the first detected fire
                self.target_x = x + w / 2  # Update target x-coordinate (center of the fire)
            else:
                self.fire_detected = False
                self.target_x = None  
            self.image_width = img.shape[1]  
    # else:
    #     print("waiting for goal to be reached")
            img_to_pub = self.bridge.cv2_to_imgmsg(img,"bgr8")
            self.image_publisher.publish(img_to_pub)
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()