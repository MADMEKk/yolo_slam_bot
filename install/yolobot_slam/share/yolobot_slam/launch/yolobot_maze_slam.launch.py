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
    namespace = LaunchConfiguration('namespace', default='')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    declare_namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot')
        
    declare_x_pose_argument = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position for robot spawning')

    declare_y_pose_argument = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position for robot spawning')
    
    # Launch joystick control
    joy_node = Node(
        package="joy",
        executable="joy_node",
        namespace=namespace
    )

    # Launch Gazebo with maze world (no namespace as it's a global resource)
    start_maze_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_gazebo, 'launch', 'start_maze_world.launch.py'),
        )
    )

    # Spawn robot with namespace and position
    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_description, 'launch', 'spawn_yolobot_launch.launch.py'),
        ),
        launch_arguments={
            'namespace': namespace,
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )     

    # Start robot control
    spawn_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_control, 'launch', 'yolobot_control.launch.py'),
        ),
        launch_arguments={'namespace': namespace}.items()
    )  

    # Start YOLO object detection
    spawn_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_recognition, 'launch', 'launch_yolov8.launch.py'),
        ),
        launch_arguments={'namespace': namespace}.items()
    )
    
    # Start SLAM Toolbox
    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_slam, 'launch', 'slam_toolbox.launch.py'),
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace
        }.items()
    )
    
    # Launch RViz with our configuration
    rviz_config_file = os.path.join(pkg_yolobot_slam, 'config', 'slam.rviz')
    
    # Start RViz (only one instance needed, no namespace)
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
        declare_namespace_argument,
        declare_x_pose_argument,
        declare_y_pose_argument,
        joy_node,
        start_maze_world,
        spawn_robot_world,
        spawn_robot_control,
        spawn_yolo,
        start_slam_toolbox,
        start_rviz
    ]) 