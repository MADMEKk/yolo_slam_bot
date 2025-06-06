#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    slam_toolbox_dir = get_package_share_directory('yolobot_slam')
    
    # Parameters file path
    slam_params_file = os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml')
    
    # Setup
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    # Add static transform publisher to ensure TF tree connectivity
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'chassis'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
            
    # Start SLAM Toolbox online async node
    start_async_slam_toolbox_node = Node(
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    # Add the various actions/entities to the launch description
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(static_tf_node)
    ld.add_action(start_async_slam_toolbox_node)

    return ld 