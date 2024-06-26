#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

from Adafruit_IO import Client, RequestError, Feed
from playsound import playsound
from rclpy.node import Node
from std_msgs.msg import String



class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")
        self.get_logger().info("Navigation Node Started")
        self.ADAFRUIT_IO_KEY = 'aio_WpIy30mhDyPUUbLtFci1IkD9QPoH'
        self.ADAFRUIT_IO_USERNAME = 'BSusheelKumar'
        self.aio = Client(self.ADAFRUIT_IO_USERNAME, self.ADAFRUIT_IO_KEY)
        self.navigator = BasicNavigator()
        self.send_search_signal = self.create_publisher(String,"/search_fire",10)
        self.check_done = self.create_subscription(String,"done_with_fire",self.done_callback,10)
        self.send_timer = self.create_timer(0.1,self.send_signal_callback)
        self.timer = self.create_timer(0.1,self.navigate_callback)
        self.initial_pose = None  # Store initial pose
        self.last_successful_goal = None 
        
        self.goal_pose = None
        self.reached_goal = False
        self.signal_sent = False
        self.proceed_to_home = False
        self.rbg_led = 23


    def send_signal_callback(self):
        pass
        # msg = String()
        # if self.reached_goal == True:  # Create a std_msgs.msg.String message object
        #     msg.data = "start_search"
        #     self.send_search_signal.publish(msg)
        # else:
        #     msg.data = "not reached goal ,waiting"
        #     self.send_search_signal.publish(msg)# Publish the std_msgs.msg.String message
    
    def done_callback(self,msg):
        if msg.data == "done":
            self.proceed_to_home = True
        else:
            self.proceed_to_home = False

    def connect_to_adafruit(self):
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.rbg_led,GPIO.OUT)
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
                GPIO.output(self.rbg_led,GPIO.HIGH)
                self.navigate_to_position(self.goal_position())

            else:
                GPIO.output(self.rbg_led,GPIO.LOW)
            
                while True:
                   
                    new_data = self.aio.receive(self.smoke.key)
                    
                    if new_data.value != '1':  
                        print('Waiting for a non-1 value from the feed...')
                    else:
                        print('Received a non-1 value from the feed!')
                        break  



    def initial_position(self):
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0023964811116456985

        self.initial_pose.pose.position.y = -0.04597257077693939
        self.initial_pose.pose.orientation.z = 0.018879440663295163
        self.initial_pose.pose.orientation.w = 0.9998217674767044

        return self.initial_pose

    def goal_position(self):
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 1.7366971969604492

        self.goal_pose.pose.position.y = 0.8917127251625061
        self.goal_pose.pose.orientation.w =0.9997994208570505
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
            msg.data = "reached_goal"
            self.send_search_signal.publish(msg)
            # if self.proceed_to_home:
            #     self.last_successful_goal = pos 
            #     if self.last_successful_goal != self.initial_pose:
            #         self.navigate_to_position(self.initial_pose)

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
