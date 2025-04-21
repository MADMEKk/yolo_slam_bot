import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default='')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.05')

    urdf = os.path.join(get_package_share_directory('yolobot_description'), 'robot/', 'yolobot.urdf')
    assert os.path.exists(urdf), "The yolobot.urdf doesnt exist in "+str(urdf)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the robot'),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial X position for robot spawning'),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial Y position for robot spawning'),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.05',
            description='Initial Z position for robot spawning (elevation)'),
        Node(
            package='yolobot_description', 
            executable='spawn_yolobot.py', 
            arguments=[urdf, namespace, x_pose, y_pose, z_pose], 
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
    ])