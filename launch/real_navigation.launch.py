import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('my_robot'), 'launch', 'bringup_launch.py')]),
        launch_arguments={'use_sim_time:=':'false',
                          'map':'room_map.yaml',
                          }.items()
)
    
    
    
    return LaunchDescription([

        nav2
    ])