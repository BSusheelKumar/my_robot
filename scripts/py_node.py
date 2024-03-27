#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

from Adafruit_IO import Client, RequestError, Feed
from playsound import playsound
from rclpy.node import Node
from std_msgs.msg import String

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
from std_msgs.msg import String



class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")
        self.get_logger().info("Navigation Node Started")
        self.ADAFRUIT_IO_KEY = 'aio_WpIy30mhDyPUUbLtFci1IkD9QPoH'
        self.ADAFRUIT_IO_USERNAME = 'BSusheelKumar'
        self.aio = Client(self.ADAFRUIT_IO_USERNAME, self.ADAFRUIT_IO_KEY)
        self.navigator = BasicNavigator()
        # self.send_search_signal = self.create_publisher(String,"/search_fire",10)
        self.send_timer = self.create_timer(0.1,self.send_signal_callback)
        self.timer = self.create_timer(0.1,self.navigate_callback)
        self.initial_pose = None  # Store initial pose
        self.last_successful_goal = None 
        
        self.goal_pose = None
        self.reached_goal = False
        self.signal_sent = False

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
        self.GPIO_setup()
        self.timer = self.create_timer(0.1, self.send_cmd_vel)
        self.relay_pin = 27  # GPIO pin connected to relay (replace with actual pin)
        self.GPIO_setup_relay()


    def send_signal_callback(self):
        pass
        # msg = String()
        # if self.reached_goal == True:  # Create a std_msgs.msg.String message object
        #     msg.data = "start_search"
        #     self.send_search_signal.publish(msg)
        # else:
        #     msg.data = "not reached goal ,waiting"
        #     self.send_search_signal.publish(msg)# Publish the std_msgs.msg.String message
    
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
        if self.reached_goal:
            if self.target_x is None or self.image_width is None:
                # Stop the robot if no fire detected
                self.stop_robot()
                return

            error = self.target_x - self.image_width / 2  # Calculate error (difference from center)
            angular_vel = -self.k_p * error  # Proportional control: angular velocity proportional to error

            # Check flame sensor for stopping near fire
            if self.is_fire_near():
                self.stop_robot()
            else:
                move = Twist()
                move.linear.x = self.linear_speed  # Set linear velocity for approach
                move.angular.z = angular_vel
                self.cmd_vel_pub.publish(move)

    def stop_robot(self):
    # Stop the robot's motion
        move = Twist()
        self.cmd_vel_pub.publish(move)

    def is_fire_near(self):
        import RPi.GPIO as GPIO
        if self.reached_goal:
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
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            fire = self.fire_cascade.detectMultiScale(img, 1.2, 5)
            print("searching for fire")
            if len(fire) > 0:  # If fire detected
                (x, y, w, h) = fire[0]  # Use the first detected fire
                self.target_x = x + w / 2  # Update target x-coordinate (center of the fire)
            else:
                self.target_x = None  # No fire detected, reset target x-coordinate
            self.image_width = img.shape[1]  # Get image width
        else:
            print("waiting for goal to be reached")

        
    def connect_to_adafruit(self):
        self.get_logger().info("Connecting To Adafruit IO")
        try:
            self.smoke = self.aio.feeds('smoke')
            data = self.aio.receive(self.smoke.key)
            print('Latest value from smoke detector: {0}'.format(data.value))
        except RequestError: # Doesn't exist, create a new feed
            feed = Feed(name="smoke")
            self.smoke = self.aio.create_feed(feed)

        while True:
            data = self.aio.receive(self.smoke.key)
            print('Latest value from smoke detector: {0}'.format(data.value))
            if data.value == '1':
                self.navigate_to_position(self.goal_position())

            else:
            # Wait for a value from the feed
                while True:
                    # Keep checking for new values
                    new_data = self.aio.receive(self.smoke.key)
                    
                    if new_data.value != '1':  # Avoid unnecessary repetition
                        print('Waiting for a non-1 value from the feed...')
                    else:
                        print('Received a non-1 value from the feed!')
                        break  # Exit the inner loop when a non-1 value is received



    def initial_position(self):
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 3.7720251083374023

        self.initial_pose.pose.position.y = -0.143662229180336
        self.initial_pose.pose.orientation.z = -0.9999558873411405
        self.initial_pose.pose.orientation.w =   0.00939272973061683
        return self.initial_pose

    def goal_position(self):
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = -0.01525208167731762

        self.goal_pose.pose.position.y =  -0.06627263873815536
        self.goal_pose.pose.orientation.w = 0.23939881632676202
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

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            self.reached_goal = True
            
            msg = String()
           
            # msg.data = "reached_goal"
            # self.send_search_signal.publish(msg)
            # self.last_successful_goal = pos 
            # if self.last_successful_goal != self.initial_pose:
            #     self.navigate_to_position(self.initial_pose)

        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.reached_goal = False
            print('Goal failed!')
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


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
