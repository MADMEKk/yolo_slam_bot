#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import select
import sys
import termios
import tty
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

# Add this import for non-interactive terminal mode
import threading

msg = """
YoloBot SLAM Teleop Control
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : increase/decrease linear velocity (default 0.2)
a/d : increase/decrease angular velocity (default 1.0)
s : force stop
q/e/z/c : move diagonally

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0),
    'e': (1, 0, -1),
    'a': (0, 0, 1),
    'd': (0, 0, -1),
    'q': (1, 0, 1),
    'x': (-1, 0, 0),
    'c': (-1, 0, 1),
    'z': (-1, 0, -1),
}

speedBindings = {
    'w': (0.1, 0),
    'x': (-0.1, 0),
    'a': (0, 0.1),
    'd': (0, -0.1),
}

# Global variable to indicate if we're in interactive mode
interactive_mode = True

def getKey(settings):
    global interactive_mode
    if not interactive_mode:
        # In non-interactive mode, just return an empty string
        # to keep the main loop running without input
        time.sleep(0.1)
        return ''
        
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    except Exception as e:
        # If there's an error, fall back to non-interactive mode
        print(f"Warning: Terminal input error: {e}")
        print("Falling back to non-interactive mode. Use ROS topics to control the robot.")
        interactive_mode = False
        return ''


def saveTerminalSettings():
    global interactive_mode
    if os.name == 'nt':
        return None
    
    try:
        return termios.tcgetattr(sys.stdin)
    except Exception as e:
        print(f"Warning: Cannot save terminal settings: {e}")
        print("Falling back to non-interactive mode. Use ROS topics to control the robot.")
        interactive_mode = False
        return None


def restoreTerminalSettings(old_settings):
    global interactive_mode
    if not interactive_mode or old_settings is None:
        return
        
    if os.name != 'nt':
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        except Exception as e:
            print(f"Warning: Cannot restore terminal settings: {e}")


def main():
    global interactive_mode
    
    settings = saveTerminalSettings()

    rclpy.init()
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, '/yolobot/cmd_vel', 10)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    # Print instructions
    node.get_logger().info(msg)
    if not interactive_mode:
        node.get_logger().info("Running in non-interactive mode.")
        node.get_logger().info("Control the robot by publishing to the /yolobot/cmd_vel topic:")
        node.get_logger().info("ros2 topic pub /yolobot/cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\"")
    
    # Create a thread for spinning the ROS node
    def spin_node():
        rclpy.spin(node)
    
    spin_thread = threading.Thread(target=spin_node)
    spin_thread.daemon = True
    spin_thread.start()

    try:
        while True:
            key = getKey(settings)
            
            if interactive_mode:
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    y = moveBindings[key][1]
                    z = moveBindings[key][2]

                    target_linear_velocity = x * 0.2
                    target_angular_velocity = z * 1.0

                elif key in speedBindings.keys():
                    if key == 'w':
                        target_linear_velocity += 0.1
                    elif key == 'x':
                        target_linear_velocity -= 0.1
                    elif key == 'a':
                        target_angular_velocity += 0.1
                    elif key == 'd':
                        target_angular_velocity -= 0.1

                    node.get_logger().info(
                        'Linear: %.2f, Angular: %.2f' % (target_linear_velocity, target_angular_velocity))

                elif key == 's':
                    target_linear_velocity = 0.0
                    target_angular_velocity = 0.0
                    node.get_logger().info('Stop!')

                else:
                    if key == '\x03':
                        break
            
            # In non-interactive mode, keep the node running but don't change velocities
            # The user can control the robot through direct ROS topic publishing
            
            twist = Twist()
            twist.linear.x = target_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_velocity
            pub.publish(twist)

    except Exception as e:
        node.get_logger().error(str(e))

    finally:
        # Stop the robot before exiting
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        
        # Restore terminal settings if possible
        restoreTerminalSettings(settings)
        
        # Shutdown ROS node
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 