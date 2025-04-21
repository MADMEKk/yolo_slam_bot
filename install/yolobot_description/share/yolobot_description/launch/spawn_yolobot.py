#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    # Read URDF from the first argument
    content = ""
    if len(sys.argv) > 1 and sys.argv[1] is not None:
        with open(sys.argv[1], 'r') as content_file:
            content = content_file.read()

    # Get namespace from second argument or use default
    namespace = ""
    robot_name = "yolobot"
    if len(sys.argv) > 2 and sys.argv[2]:
        namespace = sys.argv[2]
        robot_name = f"{namespace}"  # Strip leading/trailing slashes if any
        if robot_name.startswith('/'):
            robot_name = robot_name[1:]
        if not robot_name:
            robot_name = "yolobot"  # Fallback if namespace is just "/"
    
    # Get position from arguments or use defaults
    x_pose = 0.0
    y_pose = 0.0
    z_pose = 0.05  # Default z elevation
    
    if len(sys.argv) > 3:
        try:
            x_pose = float(sys.argv[3])
        except ValueError:
            node.get_logger().warn(f"Invalid x_pose value: {sys.argv[3]}, using default: 0.0")
    
    if len(sys.argv) > 4:
        try:
            y_pose = float(sys.argv[4])
        except ValueError:
            node.get_logger().warn(f"Invalid y_pose value: {sys.argv[4]}, using default: 0.0")
            
    if len(sys.argv) > 5:
        try:
            z_pose = float(sys.argv[5])
        except ValueError:
            node.get_logger().warn(f"Invalid z_pose value: {sys.argv[5]}, using default: 0.05")

    node.get_logger().info(f"Spawning robot '{robot_name}' with namespace '{namespace}' at position x:{x_pose}, y:{y_pose}, z:{z_pose}")

    req = SpawnEntity.Request()
    req.name = robot_name
    req.xml = content
    req.robot_namespace = namespace
    req.reference_frame = "world"
    
    # Add initial pose with the specified position
    req.initial_pose.position.x = x_pose
    req.initial_pose.position.y = y_pose
    req.initial_pose.position.z = z_pose  # Use the provided z elevation
    req.initial_pose.orientation.x = 0.0
    req.initial_pose.orientation.y = 0.0
    req.initial_pose.orientation.z = 0.0
    req.initial_pose.orientation.w = 1.0  # Unit quaternion for no rotation

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
