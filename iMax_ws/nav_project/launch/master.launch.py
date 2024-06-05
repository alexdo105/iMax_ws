import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav_project'), 'nav_localization.launch.py')
            ))
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot4_navigation'),'launch', 'nav2.launch.py')
            ))
    nav_project_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav_project'), 'nav_project.launch.py')
            ))

    return LaunchDescription([
        localization_launch,
        TimerAction(period=20.0, actions=[nav_project_launch]),
        TimerAction(period=35.0, actions=[nav2_launch])
        
    ])
