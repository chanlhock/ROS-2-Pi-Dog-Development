#!/usr/bin/env python3
"""
##########################################################################
# ROS 2 Autonomous Sunfounder Pi Dog with Raspberry Pi 5 in Ubuntu 22.04
#
# ROS 2 Dual Touch Sensor Node - Listens to touch events from main node
#  
# Copyright (c) 2026 Bernard Chan
# chanlhock@gmail.com
#
# Date           Author          Notes
# 05/05/2026     Bernard Chan    Initial release
#
# pidog_dual_touch_node.py is licensed under the GNU General Public License v3.0
# License v3.0 Permissions of this strong copyleft license are 
# conditioned on making available complete source code of licensed 
# works and modifications, which include larger works using a licensed 
# work, under the same license. Copyright and license notices must be 
# preserved. Contributors provide an express grant of patent rights.
##########################################################################
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy


class PiDogDualTouchNode(Node):
    def __init__(self):
        super().__init__('pidog_dual_touch_node')
        
        self.get_logger().info("Touch node starting - listening for touch events")
        
        # Publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        # Subscribe to touch events from main node
        self.touch_sub = self.create_subscription(
            String,
            'touch',
            self.touch_callback,
            qos_profile
        )
        
        # Re-publisher for processed touch events
        self.touch_pub = self.create_publisher(String, 'touch_event', 10)
        
        self.get_logger().info("Touch node ready - waiting for touch data")
    
    def touch_callback(self, msg: String):
        """Process and republish touch events"""
        # Parse the touch data to show human-readable format
        try:
            parts = msg.data.split(':')
            if len(parts) >= 2 and parts[0] == 'touched':
                touch_value = parts[1]
                
                # Convert touch value to readable description
                touch_desc = {
                    'L': 'Left side',
                    'R': 'Right side',
                    'LS': 'Left swipe (front to back)',
                    'RS': 'Right swipe (back to front)'
                }.get(touch_value, f'Unknown ({touch_value})')
                
                self.get_logger().info(f"👆 Touch detected: {touch_desc} ({touch_value})")
            else:
                self.get_logger().info(f"Touch received: {msg.data}")
        except Exception as e:
            self.get_logger().debug(f"Error parsing touch: {e}")
            self.get_logger().info(f"Touch received: {msg.data}")
        
        # Forward the event unchanged
        self.touch_pub.publish(msg)
    
    def shutdown(self):
        self.get_logger().info("Touch node shutting down")


def main(args=None):
    rclpy.init(args=args)
    node = PiDogDualTouchNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()