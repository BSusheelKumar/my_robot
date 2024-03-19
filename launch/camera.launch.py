import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():


    camera = Node(package='v4l2_camera', executable='v4l2_camera_node',
                        arguments=['--ros-args', '--param'
                                   , 'video_device:="/dev/video0"'],
                        output='screen')
    
    return LaunchDescription([
        camera
    ])