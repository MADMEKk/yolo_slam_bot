from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the robot'),
        Node(
            package='yolobot_recognition', 
            executable='yolov8_ros2_pt.py', 
            output='screen',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/camera/image_raw', ['/', namespace, '/camera/image_raw']),
                ('/scan', ['/', namespace, '/scan']),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ] if namespace else []
        ),
    ])