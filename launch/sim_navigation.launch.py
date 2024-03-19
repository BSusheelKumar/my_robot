import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    # slam = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('robot'), 'launch', 'slam_localization_launch.py')]),
    #         launch_arguments={'use_sim_time:=':'true'}.items()
    #     )
    
    # nav2 = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource([os.path.join(
    #     get_package_share_directory('robot'), 'launch', 'navigation_launch.py')]),
    #     launch_arguments={'use_sim_time:=':'true'}.items()
    # )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('my_robot'), 'launch', 'bringup_launch.py')]),
        launch_arguments={'use_sim_time:=':'true',
                          'map':'my_map_save.yaml',
                          }.items()
)
    
    
    
    return LaunchDescription([
        # slam,
        # nav2
        nav2
    ])