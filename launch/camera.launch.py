import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    video_device = os.environ.get('video_device', "/dev/video0")  # Handle optional video_device parameter

    return LaunchDescription([

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            arguments=[
                '--ros-args',  # Pass ROS arguments explicitly
                '--param', f'video_device:="{video_device}"'  # Set video_device parameter dynamically
            ],
        )
    ])
