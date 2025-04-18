#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    teleop_node = Node(
        package='yolobot_slam',
        executable='teleop_keyboard.py',
        name='teleop_keyboard',
        output='screen',
        emulate_tty=True,  # Important for keyboard input
    )
    
    return LaunchDescription([
        teleop_node
    ]) 