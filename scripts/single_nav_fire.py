#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from Adafruit_IO import Client, RequestError, Feed
from playsound import playsound

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import math
from geometry_msgs.msg import Twist
import time
import RPi.GPIO as GPIO

GPIO.setmode(23,GPIO.BCM)

GPIO.setmode(23,GPIO.OUT)



# Set to your Adafruit IO key.
# Remember, your key is a secret,
# so make sure not to publish it when you publish this code!
ADAFRUIT_IO_KEY = 'aio_WpIy30mhDyPUUbLtFci1IkD9QPoH'

# Set to your Adafruit IO username.
# (go to https://accounts.adafruit.com to find your username)
ADAFRUIT_IO_USERNAME = 'BSusheelKumar'


aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)


try:
    smoke = aio.feeds('smoke')
except RequestError: # Doesn't exist, create a new feed
    feed = Feed(name="smoke")
    smoke = aio.create_feed(feed)

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

                self.cls = int(box.cls[0])

                if self.cls == 0:
                    cv2.putText(img, f'{self.class_names[self.cls]} {conf}', (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
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
def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.11401252448558807
    initial_pose.pose.position.y = -0.012433409690856934
    initial_pose.pose.orientation.z = 0.005939666544692979
    initial_pose.pose.orientation.w =  0.9999823600250846
    navigator.setInitialPose(initial_pose)


    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 6.576930046081543
    goal_pose.pose.position.y = -3.7143380641937256
    goal_pose.pose.orientation.w = 0.7019733675374062

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)
    while True:
        data = aio.receive(smoke.key)
        print('Latest value from smoke detector: {0}'.format(data.value))
        if data.value == '1':
            # playsound("/home/bsusheelkumar/final_year_project/src/my_robot/jaldhi waha se hato.mp3")
            navigator.goToPose(goal_pose)

            i = 0
            while not navigator.isTaskComplete():
                ################################################
                #
                # Implement some code here for your application!
                #
                ################################################

                # Do something with the feedback
                i = i + 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')

                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        navigator.cancelTask()

                    # Some navigation request change to demo preemption
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                        goal_pose.pose.position.x != -3.0
                        navigator.goToPose(goal_pose)

            # Do something depending on the return code
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

            navigator.lifecycleShutdown()

            exit(0)
        else:
            # Wait for a value from the feed
            while True:
                # Keep checking for new values
                new_data = aio.receive(smoke.key)
                
                if new_data.value != '1':  # Avoid unnecessary repetition
                    print('Waiting for a non-1 value from the feed...')
                else:
                    print('Received a non-1 value from the feed!')
                    break  # Exit the inner loop when a non-1 value is received
    rclpy.shutdown()

if __name__ == '__main__':
    main()