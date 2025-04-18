#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import math

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        
        # Subscribe to LaserScan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/yolobot/scan',
            self.scan_callback,
            10)
        
        # Publishers for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, '/lidar_visualization', 10)
        self.history_pub = self.create_publisher(MarkerArray, '/lidar_history', 10)
        
        # For storing history
        self.history = []
        self.max_history_points = 5000  # Maximum history points to store
        self.history_count = 0
        self.history_skip = 5  # Only store every Nth scan to reduce density
        
        # Create a timer for publishing history
        self.history_timer = self.create_timer(1.0, self.publish_history)
        
        self.get_logger().info('LiDAR Visualizer started - Listening on /yolobot/scan')
    
    def scan_callback(self, msg):
        # Points marker
        self.visualize_current_scan(msg)
        
        # Store in history (sparsely)
        self.history_count += 1
        if self.history_count % self.history_skip == 0:
            self.store_in_history(msg)
    
    def visualize_current_scan(self, msg):
        marker_array = MarkerArray()
        
        # Create point cloud marker
        points_marker = Marker()
        points_marker.header = msg.header
        points_marker.ns = "lidar_points"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.05  # Point size
        points_marker.scale.y = 0.05
        points_marker.color.r = 1.0
        points_marker.color.g = 0.0
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0
        points_marker.lifetime.sec = 0
        points_marker.lifetime.nanosec = 500000000  # 0.5 second lifetime
        
        # Visual rays marker
        rays_marker = Marker()
        rays_marker.header = msg.header
        rays_marker.ns = "lidar_rays"
        rays_marker.id = 1
        rays_marker.type = Marker.LINE_LIST
        rays_marker.action = Marker.ADD
        rays_marker.scale.x = 0.01  # Line width
        rays_marker.color.r = 0.0
        rays_marker.color.g = 1.0
        rays_marker.color.b = 0.0
        rays_marker.color.a = 0.3
        rays_marker.lifetime.sec = 0
        rays_marker.lifetime.nanosec = 500000000  # 0.5 second lifetime
        
        # Add origin point for rays
        origin = Point()
        origin.x = 0.0
        origin.y = 0.0
        origin.z = 0.0
        
        for i, range_val in enumerate(msg.ranges):
            if range_val < msg.range_max and range_val > msg.range_min:
                angle = msg.angle_min + i * msg.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                
                # Add point
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.1  # Slightly above the ground
                points_marker.points.append(point)
                
                # Add ray
                rays_marker.points.append(origin)
                rays_marker.points.append(point)
        
        marker_array.markers.append(points_marker)
        marker_array.markers.append(rays_marker)
        self.marker_pub.publish(marker_array)
    
    def store_in_history(self, msg):
        for i, range_val in enumerate(msg.ranges):
            # Store only every few points to avoid too much data
            if i % 5 == 0 and range_val < msg.range_max and range_val > msg.range_min:
                angle = msg.angle_min + i * msg.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                
                # Store the point in history with frame ID and timestamp
                self.history.append({
                    'x': x, 
                    'y': y, 
                    'z': 0.05,
                    'frame_id': msg.header.frame_id,
                    'stamp': msg.header.stamp
                })
        
        # Trim history if too large
        if len(self.history) > self.max_history_points:
            self.history = self.history[-self.max_history_points:]
    
    def publish_history(self):
        if not self.history:
            return
            
        marker_array = MarkerArray()
        
        # Create marker for history points
        history_marker = Marker()
        history_marker.header.frame_id = self.history[0]['frame_id']
        history_marker.header.stamp = self.get_clock().now().to_msg()
        history_marker.ns = "lidar_history"
        history_marker.id = 2
        history_marker.type = Marker.POINTS
        history_marker.action = Marker.ADD
        history_marker.scale.x = 0.02  # Smaller points for history
        history_marker.scale.y = 0.02
        history_marker.color.r = 0.0
        history_marker.color.g = 0.0
        history_marker.color.b = 1.0
        history_marker.color.a = 0.5
        
        # Add all history points
        for point in self.history:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = point['z']
            history_marker.points.append(p)
        
        marker_array.markers.append(history_marker)
        self.history_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    lidar_visualizer = LidarVisualizer()
    
    try:
        rclpy.spin(lidar_visualizer)
    except KeyboardInterrupt:
        pass
    
    lidar_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 