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

    content = ""
    if sys.argv[1] is not None:
        with open(sys.argv[1], 'r') as content_file:
            content = content_file.read()

    req = SpawnEntity.Request()
    req.name = "yolobot"
    req.xml = content
    req.robot_namespace = ""
    req.reference_frame = "world"
    
    # Add initial pose to ensure the robot spawns correctly
    req.initial_pose.position.x = 0.0
    req.initial_pose.position.y = 0.0
    req.initial_pose.position.z = 0.05  # Slight elevation to prevent clipping
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
