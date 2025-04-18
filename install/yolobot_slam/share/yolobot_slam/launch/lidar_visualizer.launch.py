#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolobot_slam',
            executable='lidar_visualizer.py',
            name='lidar_visualizer',
            output='screen'
        )
    ]) 