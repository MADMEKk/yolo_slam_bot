from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('yolobot_slam')
    
    return LaunchDescription([
        Node(
            package='yolobot_slam',
            executable='simple_teleop.py',
            name='simple_teleop',
            output='screen'
        )
    ]) 