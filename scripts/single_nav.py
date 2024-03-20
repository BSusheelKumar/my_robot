#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

from Adafruit_IO import Client, RequestError, Feed
from playsound import playsound

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


def main():
    rclpy.init()

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
            playsound("/home/bsusheelkumar/final_year_project/src/my_robot/jaldhi waha se hato.mp3")
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

if __name__ == '__main__':
    main()