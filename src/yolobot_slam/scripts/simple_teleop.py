#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

# This is a simplified teleop script that doesn't rely on termios

msg = """
YoloBot SLAM Simple Teleop Control
----------------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : forward/backward
a/d : rotate left/right
q/e/z/c : move diagonally
s : force stop

Enter the key and press ENTER to send the command.
Type 'quit' to exit.
"""

class SimpleKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        self.publisher = self.create_publisher(Twist, '/yolobot/cmd_vel', 10)
        self.get_logger().info(msg)
        
        # Default velocities
        self.linear_speed = 0.2
        self.angular_speed = 1.0
        
        # Movement mappings 
        self.moveBindings = {
            'w': (1, 0, 0),    # Forward
            'x': (-1, 0, 0),   # Backward
            'a': (0, 0, 1),    # Rotate left
            'd': (0, 0, -1),   # Rotate right
            'q': (1, 0, 1),    # Forward + rotate left
            'e': (1, 0, -1),   # Forward + rotate right
            'z': (-1, 0, -1),  # Backward + rotate right
            'c': (-1, 0, 1),   # Backward + rotate left
            's': (0, 0, 0),    # Stop
        }
        
    def process_key(self, key):
        twist = Twist()
        
        if key in self.moveBindings:
            x, y, z = self.moveBindings[key]
            twist.linear.x = x * self.linear_speed
            twist.linear.y = y * self.linear_speed
            twist.angular.z = z * self.angular_speed
            
            move_desc = ""
            if x > 0:
                move_desc += "forward "
            elif x < 0:
                move_desc += "backward "
                
            if z > 0:
                move_desc += "rotate left"
            elif z < 0:
                move_desc += "rotate right"
            elif x == 0 and z == 0:
                move_desc = "stop"
                
            self.get_logger().info(f"Movement: {move_desc}")
            
        elif key == 'i':
            self.linear_speed += 0.1
            self.get_logger().info(f"Linear speed: {self.linear_speed:.1f}")
            return
            
        elif key == 'k':
            self.linear_speed -= 0.1
            if self.linear_speed < 0:
                self.linear_speed = 0.0
            self.get_logger().info(f"Linear speed: {self.linear_speed:.1f}")
            return
            
        elif key == 'j':
            self.angular_speed += 0.1
            self.get_logger().info(f"Angular speed: {self.angular_speed:.1f}")
            return
            
        elif key == 'l':
            self.angular_speed -= 0.1
            if self.angular_speed < 0:
                self.angular_speed = 0.0
            self.get_logger().info(f"Angular speed: {self.angular_speed:.1f}")
            return
        
        else:
            # If key not recognized, stop the robot
            self.get_logger().info(f"Unknown command: '{key}'. Robot stopped.")
            
        # Publish the command
        self.publisher.publish(twist)
        
        # For stopping, also log the speed
        if key == 's':
            self.get_logger().info(f"Robot stopped. Linear speed: {self.linear_speed:.1f}, Angular speed: {self.angular_speed:.1f}")

def main():
    rclpy.init()
    teleop = SimpleKeyboardTeleop()
    
    # Create a thread for spinning the ROS node
    def spin_node():
        rclpy.spin(teleop)
    
    spin_thread = threading.Thread(target=spin_node)
    spin_thread.daemon = True
    spin_thread.start()
    
    try:
        while True:
            print("\nEnter command (w/a/s/d/q/e/z/c or 'quit' to exit): ", end='', flush=True)
            key = input().strip().lower()
            
            if key == 'quit':
                break
                
            if key:  # Only process if a key was actually entered
                teleop.process_key(key)
            
            # Sleep briefly to allow messages to be processed
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
    
    finally:
        # Stop the robot before exiting
        stop_twist = Twist()
        teleop.publisher.publish(stop_twist)
        teleop.get_logger().info("Stopping robot and shutting down.")
        
        # Properly shut down the node
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 