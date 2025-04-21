#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


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
    
    # === GLOBAL RESOURCES (launched once) ===
    
    # Launch Gazebo with maze world (only need to do this once)
    start_maze_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_gazebo, 'launch', 'start_maze_world.launch.py'),
        )
    )

    # Launch RViz with multi-robot configuration (only need one instance)
    rviz_config_file = os.path.join(pkg_yolobot_slam, 'config', 'multi_robot_slam.rviz')
    
    # Use the default slam.rviz if multi_robot_slam.rviz doesn't exist
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(pkg_yolobot_slam, 'config', 'slam.rviz')
    
    # Start RViz (single instance)
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Define robot configurations - place robots at safe locations in the maze
    # Using specific coordinates that are known to be clear of obstacles
    robot_configs = [
        {
            'namespace': 'robot1',
            'x_pose': '-2.0',  # First robot in a likely open area
            'y_pose': '-2.0',
            'z_pose': '0.10',  # Slightly elevated to avoid floor clipping
        },
        {
            'namespace': 'robot2',
            'x_pose': '2.0',   # Second robot in another likely open area
            'y_pose': '2.0',
            'z_pose': '0.10',  # Slightly elevated to avoid floor clipping
        }
    ]

    # === PER-ROBOT RESOURCES (launched for each robot) ===
    
    # Create launch description for all robots
    robot_launches = []
    
    for robot_config in robot_configs:
        namespace = robot_config['namespace']
        x_pose = robot_config['x_pose']
        y_pose = robot_config['y_pose']
        z_pose = robot_config['z_pose']
        
        # Group actions for this robot with its namespace
        with_namespace = GroupAction([
            # Push the namespace
            PushRosNamespace(namespace),
            
            # 1. Spawn robot with namespace and position
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_yolobot_description, 'launch', 'spawn_yolobot_launch.launch.py'),
                ),
                launch_arguments={
                    'namespace': namespace,
                    'x_pose': x_pose,
                    'y_pose': y_pose,
                    'z_pose': z_pose,
                    'use_sim_time': use_sim_time,
                }.items()
            ),
            
            # 2. Start robot control
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_yolobot_control, 'launch', 'yolobot_control.launch.py'),
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                }.items()
            ),
            
            # 3. Start YOLO object detection
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_yolobot_recognition, 'launch', 'launch_yolov8.launch.py'),
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                }.items()
            ),
            
            # 4. Start SLAM Toolbox
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_yolobot_slam, 'launch', 'slam_toolbox.launch.py'),
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                }.items()
            ),
            
            # 5. Launch joystick control (if needed)
            Node(
                package="joy",
                executable="joy_node",
                namespace=namespace
            ),
        ])
        
        robot_launches.append(with_namespace)

    # === LAUNCH DESCRIPTION ===
    
    # Combine everything into a launch description
    ld = LaunchDescription()
    
    # Add the declare arguments
    ld.add_action(declare_use_sim_time_argument)
    
    # Add global resources (once)
    ld.add_action(start_maze_world)
    
    # Add all robots
    for launch_item in robot_launches:
        ld.add_action(launch_item)
    
    # Add RViz (once)
    ld.add_action(start_rviz)
    
    return ld 