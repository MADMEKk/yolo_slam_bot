import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the robot'),
        Node(
            package='yolobot_control', 
            executable='robot_control.py', 
            output='screen',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])