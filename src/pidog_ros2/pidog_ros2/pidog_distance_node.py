#!/usr/bin/env python3
"""
##########################################################################
# ROS 2 Autonomous Sunfounder Pi Dog with Raspberry Pi 5 in Ubuntu 22.04
#
# ROS 2 Distance Sensor Node for Pi Dog
# NO HARDWARE ACCESS - receives data from main node
#  
# Copyright (c) 2026 Bernard Chan
# chanlhock@gmail.com
#
# Date           Author          Notes
# 05/05/2026     Bernard Chan    Initial release
#
# pidog_distance_node.py is licensed under the GNU General Public 
# License v3.0 Permissions of this strong copyleft license are 
# conditioned on making available complete source code of licensed 
# works and modifications, which include larger works using a licensed 
# work, under the same license. Copyright and license notices must be 
# preserved. Contributors provide an express grant of patent rights.
##########################################################################
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32, String

from collections import deque 

class PiDogDistanceNode(Node):
    def __init__(self):
        super().__init__('pidog_distance_node')
        
        self.get_logger().info("Distance node starting (NO hardware initialization)")
        
        # Parameters
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('moving_average_window', 5)
        
        self.publish_freq = self.get_parameter('publish_frequency').value
        self.moving_window = self.get_parameter('moving_average_window').value
        
        # Buffer for moving average
        self.distance_buffer = deque(maxlen=self.moving_window)
        
        # Publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.distance_filtered_pub = self.create_publisher(Float32, 'distance_filtered', qos_profile)
        self.distance_detailed_pub = self.create_publisher(String, 'distance/detailed', qos_profile)
        
        # Subscriber to main node's distance topic
        self.distance_sub = self.create_subscription(
            Float32,
            'distance',
            self.distance_callback,
            qos_profile
        )

        # Timer for publishing filtered data
        timer_period = 1.0 / self.publish_freq
        self.timer = self.create_timer(timer_period, self.publish_filtered)
        
        self.get_logger().info(f"Distance node ready (publishing at {self.publish_freq} Hz)")
    
    def distance_callback(self, msg: Float32):
        """Receive raw distance from main node"""
        self.get_logger().debug(f"Received distance: {msg.data:.2f} cm")
        self.distance_buffer.append(msg.data)
    
    def publish_filtered(self):
        """Publish filtered distance"""
        if len(self.distance_buffer) > 0:
            filtered = sum(self.distance_buffer) / len(self.distance_buffer)
            self.distance_filtered_pub.publish(Float32(data=filtered))
    
    def shutdown(self):
        self.get_logger().info("Distance node shutting down")


def main(args=None):
    rclpy.init(args=args)
    node = PiDogDistanceNode()
    
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
