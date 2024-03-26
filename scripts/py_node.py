#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

from Adafruit_IO import Client, RequestError, Feed
from playsound import playsound
from std_msgs.msg import String
import RPi.GPIO as GPIO

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Testing fire tracking with stopping")
        self.camera_sub = self.create_subscription(Image, "/image_raw", self.camera_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.bridge = CvBridge()
        self.fire_cascade = cv2.CascadeClassifier('/home/robot/final_year_project/src/my_robot/fire_detection.xml')
        self.target_x = None
        self.image_width = None
        self.k_p = 0.001  # Proportional control gain
        self.linear_speed = 0.2  # Linear velocity for approach
        self.approach_distance = 0.5  # Minimum distance for stopping (in meters)
        self.flame_pin = 4  # GPIO pin connected to flame sensor (replace with actual pin)
        self.relay_pin = 18
        self.GPIO_setup()
        self.timer = self.create_timer(0.1, self.send_cmd_vel)
        self.get_logger().info("Navigation Node Started")
        self.ADAFRUIT_IO_KEY = 'aio_WpIy30mhDyPUUbLtFci1IkD9QPoH'
        self.ADAFRUIT_IO_USERNAME = 'BSusheelKumar'
        self.aio = Client(self.ADAFRUIT_IO_USERNAME, self.ADAFRUIT_IO_KEY)
        self.navigator = BasicNavigator()
        self.timer = self.create_timer(0.1,self.navigate_callback)
        self.initial_pose = None  # Store initial pose
        self.last_successful_goal = None 
        self.reached_fire_goal = False


    def connect_to_adafruit(self):
        self.get_logger().info("Connecting To Adafruit IO")
        try:
            self.smoke = self.aio.feeds('smoke')
            data = self.aio.receive(self.smoke.key)
            print('Latest value from smoke detector: {0}'.format(data.value))
            if data.value == '1':
                # playsound("/home/bsusheelkumar/final_year_project/src/my_robot/jaldhi waha se hato.mp3")
                self.navigate_to_position(self.goal_position())
        except RequestError: # Doesn't exist, create a new feed
            feed = Feed(name="smoke")
            self.smoke = self.aio.create_feed(feed)

    def initial_position(self):
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = -0.9338361024856567
        self.initial_pose.pose.position.y = -1.6912544965744019
        self.initial_pose.pose.orientation.z = -0.06557356597535592
        self.initial_pose.pose.orientation.w =  0.9978477376059314
        return self.initial_pose
    
    def goal_position(self):
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = -7.851898670196533
        self.goal_pose.pose.position.y = 2.2073423862457275
        self.goal_pose.pose.orientation.w = 0.7362738136290178
        return self.goal_pose

    def navigate_to_position(self, pos):
        self.navigator.goToPose(pos)
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    pos.pose.position.x != -3.0
                    self.navigator.goToPose(pos)
                    self.navigator

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            self.reached_fire_goal = True
            
            # self.last_successful_goal = pos 
            # if self.last_successful_goal != self.initial_pose:
            #     self.navigate_to_position(self.initial_pose)

        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            self.reached_fire_goal = False
            # if self.last_successful_goal != self.initial_pose:
            #     self.navigate_to_position(self.initial_pose)
        else:
            print('Goal has an invalid return status!')

        # exit(0)

    def navigate_callback(self):
        self.initial_position()
        self.navigator.setInitialPose(self.initial_position())
        self.navigator.waitUntilNav2Active()
        self.connect_to_adafruit()

    def GPIO_setup(self):
        import RPi.GPIO as GPIO  # Import GPIO library here
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.flame_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.relay_pin,GPIO.OUT)

    def send_cmd_vel(self):
        if self.target_x is None or self.image_width is None:
            # Stop the robot if no fire detected
            self.stop_robot()
            return

        error = self.target_x - self.image_width / 2  # Calculate error (difference from center)
        angular_vel = -self.k_p * error  # Proportional control: angular velocity proportional to error

        # Check flame sensor for stopping near fire
        if self.is_fire_near():
            self.stop_robot()
            print("Fire is Near me")
            self.fire_extinguish(True)
        else:
            self.fire_extinguish(False)
            move = Twist()
            move.linear.x = self.linear_speed  # Set linear velocity for approach
            move.angular.z = angular_vel
            self.cmd_vel_pub.publish(move)

    def stop_robot(self):
        # Stop the robot's motion
        move = Twist()
        self.cmd_vel_pub.publish(move)

    def is_fire_near(self):
        import RPi.GPIO as GPIO  # Import GPIO library here
        flame_detected = GPIO.input(self.flame_pin)
        return flame_detected == 0  # Check for LOW signal indicating fire
    
    def fire_extinguish(self,act=False):
        import RPi.GPIO as GPIO
        if act:
            print("Turning On Pump")
            GPIO.output(self.relay_pin,GPIO.LOW)
        else:
            print("Pump not turned On")
            GPIO.output(self.relay_pin,GPIO.HIGH)
    def camera_callback(self, data):
        if self.reached_fire_goal:
            print("Running Image Processing")
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            fire = self.fire_cascade.detectMultiScale(img, 1.2, 5)
            if len(fire) > 0:  # If fire detected
                print("fire Detected")
                (x, y, w, h) = fire[0]  # Use the first detected fire
                self.target_x = x + w / 2  # Update target x-coordinate (center of the fire)
            else:
                self.target_x = None  # No fire detected, reset target x-coordinate
            self.image_width = img.shape[1]  # Get image width
        else:
            print("Goal has not reached")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
