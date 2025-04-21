#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_remappings(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    if namespace:
        return [
            ('/scan', f'/{namespace}/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('/map', f'/{namespace}/map'),
            ('/map_metadata', f'/{namespace}/map_metadata')
        ]
    else:
        return []


def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    slam_toolbox_dir = get_package_share_directory('yolobot_slam')
    
    # Parameters file path
    slam_params_file = os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml')
    
    # Get parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    
    # Add static transform publisher to ensure TF tree connectivity
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        namespace=namespace,
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'chassis'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
            
    # Start SLAM Toolbox online async node
    start_async_slam_toolbox_node = Node(
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        remappings=get_remappings(context)
    )
    
    return [static_tf_node, start_async_slam_toolbox_node]


def generate_launch_description():
    # Declare arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
        
    declare_namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot')
    
    # Use OpaqueFunction to resolve LaunchConfiguration values at runtime
    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_namespace_argument,
        OpaqueFunction(function=launch_setup)
    ]) 