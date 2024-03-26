#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

from Adafruit_IO import Client, RequestError, Feed
from playsound import playsound
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")
        self.get_logger().info("Navigation Node Started")
        self.ADAFRUIT_IO_KEY = 'aio_WpIy30mhDyPUUbLtFci1IkD9QPoH'
        self.ADAFRUIT_IO_USERNAME = 'BSusheelKumar'
        self.aio = Client(self.ADAFRUIT_IO_USERNAME, self.ADAFRUIT_IO_KEY)
        self.navigator = BasicNavigator()
        self.timer = self.create_timer(0.1,self.navigate_callback)
        self.initial_pose = None  # Store initial pose
        self.last_successful_goal = None 
        self.send_search_signal = self.create_publisher(String,"/search_fire",10)

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
            self.last_successful_goal = pos 
            if self.last_successful_goal != self.initial_pose:
                self.navigate_to_position(self.initial_pose)

        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            if self.last_successful_goal != self.initial_pose:
                self.navigate_to_position(self.initial_pose)
        else:
            print('Goal has an invalid return status!')

        exit(0)

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
