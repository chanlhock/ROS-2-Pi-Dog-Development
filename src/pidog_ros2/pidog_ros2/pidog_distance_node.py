#!/usr/bin/env python3
"""
##########################################################################
# ROS 2 Autonomous Sunfounder Pi Dog with Raspberry Pi 5 in Ubuntu 22.04
#
# ROS 2 Distance Sensor Node for Pi Dog (WITH ADVANCED FILTERING)
# Receives raw distance data and publishes filtered results
#  
# Copyright (c) 2026 Bernard Chan
# chanlhock@gmail.com
#
# Date           Author          Notes
# 05/05/2026     Bernard Chan    Initial release
# 16/05/2026     Bernard Chan    Added moving average + outlier rejection
# 18/05/2026     Bernard Chan    Fixed aggressive filtering (94% rejection issue)
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
import numpy as np
import random
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32, String

class PiDogDistanceNode(Node):
    def __init__(self):
        super().__init__('pidog_distance_node')
        
        self.get_logger().info("Distance node starting with advanced filtering")
        
        # Parameters
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('moving_average_window', 5)
        self.declare_parameter('outlier_threshold_cm', 50.0)  # INCREASED from 30 to 50
        self.declare_parameter('min_readings_for_filter', 3)
        
        self.publish_freq = self.get_parameter('publish_frequency').value
        self.moving_window = self.get_parameter('moving_average_window').value
        self.outlier_threshold = self.get_parameter('outlier_threshold_cm').value
        self.min_readings = self.get_parameter('min_readings_for_filter').value
        
        # Buffers for filtering
        self.raw_buffer = deque(maxlen=self.moving_window)  # Raw readings
        self.filtered_buffer = deque(maxlen=10)  # History of filtered values for stability checking
        
        # ADDED: Store last valid distance for fallback
        self.last_valid_distance = 999.0
        
        # ADDED: Max change per reading (less aggressive)
        self.max_change_per_reading = 100.0  # cm - ultrasonic can have large jumps
        
        # Publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Filtered distance (for obstacle avoidance)
        self.distance_filtered_pub = self.create_publisher(Float32, 'distance/filtered', qos_profile)
        
        # Stable distance (with additional smoothing - for display/logging)
        self.distance_stable_pub = self.create_publisher(Float32, 'distance/stable', qos_profile)
        
        # Detailed info for debugging
        self.distance_detailed_pub = self.create_publisher(String, 'distance/detailed', qos_profile)
        
        # Subscriber to main node's raw distance topic
        self.distance_sub = self.create_subscription(
            Float32,
            'distance/raw',
            self.distance_callback,
            qos_profile
        )

        # Timer for publishing filtered data
        timer_period = 1.0 / self.publish_freq
        self.timer = self.create_timer(timer_period, self.publish_filtered)
        
        # Statistics for monitoring
        self.rejected_outliers = 0
        self.total_readings = 0
        
        self.get_logger().info(f"Distance node ready with advanced filtering:")
        self.get_logger().info(f"  - Window size: {self.moving_window}")
        self.get_logger().info(f"  - Outlier threshold: {self.outlier_threshold} cm")
        self.get_logger().info(f"  - Max change per reading: {self.max_change_per_reading} cm")
        self.get_logger().info(f"  - Publishing at {self.publish_freq} Hz")
    
    def reject_outliers(self, readings):
        """
        Simplified threshold-based outlier rejection (less aggressive than IQR)
        """
        if len(readings) < self.min_readings:
            return readings
        
        # Calculate median (more robust than mean)
        sorted_readings = sorted(readings)
        median = sorted_readings[len(sorted_readings)//2]
        
        # Only reject readings that are extremely far from median
        filtered = [d for d in readings if abs(d - median) <= self.outlier_threshold]
        
        # Log outlier rejection
        rejected = len(readings) - len(filtered)
        if rejected > 0:
            self.rejected_outliers += rejected
            self.get_logger().debug(f"Rejected {rejected} outlier(s) - threshold {self.outlier_threshold}cm from median {median:.1f}")
        
        # If we rejected too many, keep the median only
        if len(filtered) < self.min_readings:
            self.get_logger().debug(f"Too many outliers ({rejected}/{len(readings)}), using median only")
            return [median]
        
        return filtered
    
    def moving_average(self, readings):
        """Calculate moving average of filtered readings"""
        if not readings:
            return self.last_valid_distance
        return sum(readings) / len(readings)
    
    def apply_temporal_consistency(self, current_filtered):
        """
        Less aggressive temporal consistency - allow gradual changes
        """
        if len(self.filtered_buffer) < 2:
            return current_filtered
        
        last_value = self.filtered_buffer[-1]
        
        # Allow gradual changes - use weighted average for large jumps
        if abs(current_filtered - last_value) > self.max_change_per_reading:
            self.get_logger().debug(f"Large jump detected: {last_value:.1f} -> {current_filtered:.1f} cm, applying smoothing")
            # Return weighted average instead of last value
            return (last_value * 0.6 + current_filtered * 0.4)
        
        return current_filtered
    
    def distance_callback(self, msg: Float32):
        """Receive raw distance from main node and filter"""
        distance = msg.data
        
        # Basic validity check
        if 2 <= distance <= 400:  # Valid range for ultrasonic sensor
            self.total_readings += 1
            self.raw_buffer.append(distance)
            
            self.get_logger().debug(f"Raw distance received: {distance:.2f} cm")
            
            # Log outlier statistics periodically
            if self.total_readings % 50 == 0 and self.total_readings > 0:
                rejection_rate = (self.rejected_outliers / self.total_readings) * 100
                self.get_logger().info(f"📊 Filter stats: {self.rejected_outliers}/{self.total_readings} "
                                     f"({rejection_rate:.1f}% rejected)")
        else:
            self.get_logger().warn(f"Invalid raw distance ignored: {distance:.2f} cm")
    
    def publish_filtered(self):
        """Apply combined filtering and publish results"""
        if len(self.raw_buffer) < self.min_readings:
            # Not enough data yet - use last valid if available
            if self.last_valid_distance < 999:
                self.distance_filtered_pub.publish(Float32(data=self.last_valid_distance))
                self.distance_stable_pub.publish(Float32(data=self.last_valid_distance))
            return
        
        # Step 1: Get current raw readings
        current_readings = list(self.raw_buffer)
        
        # Step 2: Reject outliers (now less aggressive)
        clean_readings = self.reject_outliers(current_readings)
        
        # Step 3: Calculate moving average on clean readings
        filtered_distance = self.moving_average(clean_readings)
        
        # Step 4: Apply temporal consistency check
        stable_distance = self.apply_temporal_consistency(filtered_distance)
        
        # Store last valid
        self.last_valid_distance = stable_distance
        
        # Step 5: Store for next consistency check
        self.filtered_buffer.append(stable_distance)
        
        # Step 6: Publish both filtered and stable distances
        self.distance_filtered_pub.publish(Float32(data=filtered_distance))
        self.distance_stable_pub.publish(Float32(data=stable_distance))
        
        # Step 7: Publish detailed info for debugging (reduced frequency)
        if random.randint(1, 20) == 1:
            raw_min = min(current_readings)
            raw_max = max(current_readings)
            detail_msg = (f"raw_count:{len(current_readings)}:"
                         f"clean_count:{len(clean_readings)}:"
                         f"filtered:{filtered_distance:.1f}:"
                         f"stable:{stable_distance:.1f}:"
                         f"raw_min:{raw_min:.1f}:"
                         f"raw_max:{raw_max:.1f}")
            self.distance_detailed_pub.publish(String(data=detail_msg))
        
        # Log at reduced rate
        if random.randint(1, 15) == 1:
            raw_range = f"{min(current_readings):.0f}-{max(current_readings):.0f}"
            self.get_logger().info(f"📏 Distance: raw={raw_range}cm → filtered={filtered_distance:.0f}cm → stable={stable_distance:.0f}cm")
    
    def shutdown(self):
        self.get_logger().info(f"Distance node shutting down. Final stats: "
                              f"{self.rejected_outliers}/{self.total_readings} outliers rejected "
                              f"({(self.rejected_outliers/self.total_readings*100 if self.total_readings>0 else 0):.1f}%)")


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
