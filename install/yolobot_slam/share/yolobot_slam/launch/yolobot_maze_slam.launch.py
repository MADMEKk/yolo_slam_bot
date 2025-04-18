#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get packages directories
    pkg_yolobot_gazebo = get_package_share_directory('yolobot_gazebo')
    pkg_yolobot_description = get_package_share_directory('yolobot_description')
    pkg_yolobot_control = get_package_share_directory('yolobot_control')
    pkg_yolobot_recognition = get_package_share_directory('yolobot_recognition')
    pkg_yolobot_slam = get_package_share_directory('yolobot_slam')

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    # Launch joystick control
    joy_node = Node(
        package="joy",
        executable="joy_node"
    )

    # Launch Gazebo with maze world
    start_maze_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_gazebo, 'launch', 'start_maze_world.launch.py'),
        )
    )

    # Spawn robot
    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_description, 'launch', 'spawn_yolobot_launch.launch.py'),
        )
    )     

    # Start robot control
    spawn_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_control, 'launch', 'yolobot_control.launch.py'),
        )
    )  

    # Start YOLO object detection
    spawn_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_recognition, 'launch', 'launch_yolov8.launch.py'),
        )
    )
    
    # Start SLAM Toolbox
    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_slam, 'launch', 'slam_toolbox.launch.py'),
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Launch RViz with our configuration
    rviz_config_file = os.path.join(pkg_yolobot_slam, 'config', 'slam.rviz')
    
    # Start RViz
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        joy_node,
        start_maze_world,
        spawn_robot_world,
        spawn_robot_control,
        spawn_yolo,
        start_slam_toolbox,
        start_rviz
    ]) 