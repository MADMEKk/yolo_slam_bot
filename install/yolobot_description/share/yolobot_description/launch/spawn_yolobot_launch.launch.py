import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default='')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.05')

    # Get the xacro file path
    urdf_xacro = os.path.join(get_package_share_directory('yolobot_description'), 'robot/', 'yolobot.urdf.xacro')
    assert os.path.exists(urdf_xacro), f"The yolobot.urdf.xacro doesn't exist in {urdf_xacro}"
    
    # Use xacro to process the file
    robot_description_content = Command([
        'xacro', ' ', urdf_xacro,
        ' ', 'robot_name:=', namespace,
        ' ', 'prefix:=', namespace, '/' if namespace else ''
    ])
    
    # Create a temporary urdf file for spawning
    robot_description = {'robot_description': robot_description_content}

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
        
        # Publish the robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]),
            
        # Spawn the robot with the provided parameters
        Node(
            package='yolobot_description', 
            executable='spawn_yolobot.py', 
            arguments=['-', namespace, x_pose, y_pose, z_pose, robot_description_content], 
            output='screen'),
    ])