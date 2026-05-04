#!/usr/bin/env python3
"""
ROS 2 Distance Sensor Node for Pi Dog
Publishes ultrasonic distance measurements
Uses standard ROS 2 message types (no custom imports required)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Standard ROS 2 imports instead of custom ones
from std_msgs.msg import Float32
from std_msgs.msg import String

import sys
import time
import threading
import random
from collections import deque


class PiDogDistanceNode(Node):
    def __init__(self):
        super().__init__('pidog_distance_node')
        
        # Parameters
        self.declare_parameter('publish_frequency', 10.0)  # Hz
        self.declare_parameter('distance_timeout', 2.0)  # seconds
        self.declare_parameter('moving_average_window', 5)
        self.declare_parameter('enable_head_scanning', False)  # Disabled by default
        
        self.publish_freq = self.get_parameter('publish_frequency').value
        self.distance_timeout = self.get_parameter('distance_timeout').value
        self.moving_window = self.get_parameter('moving_average_window').value
        self.enable_head_scanning = self.get_parameter('enable_head_scanning').value
        
        # Distance buffer for moving average
        self.distance_buffer = deque(maxlen=self.moving_window)
        
        # ROS 2 Publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Simple distance publisher (Float32 - easy to use)
        self.distance_pub = self.create_publisher(
            Float32,
            'distance',
            qos_profile
        )
        
        # Detailed distance publisher (String with quality info)
        self.distance_detailed_pub = self.create_publisher(
            String,
            'distance/detailed',
            qos_profile
        )
        
        # Try to import PiDog
        self.dog = None
        try:
            sys.path.append('/usr/local/lib/python3.10/dist-packages')
            from pidog import Pidog
            self.dog = Pidog()
            self.get_logger().info("Distance sensor initialized successfully")
        except ImportError:
            self.get_logger().warning("PiDog library not available - running in simulation")
        
        # Timer for publishing
        timer_period = 1.0 / self.publish_freq
        self.timer = self.create_timer(timer_period, self.publish_distance)
        
        # Start head scanning thread for 360-degree awareness (optional)
        self.scanning = False
        self.scan_thread = None
        if self.enable_head_scanning and self.dog:
            self.start_head_scanning()
        
        self.get_logger().info("PiDog Distance Node Ready")
        self.get_logger().info(f"Publishing to: /distance (Float32) and /distance/detailed (String)")
    
    def read_distance(self):
        """Read distance from ultrasonic sensor with filtering"""
        if self.dog is None:
            # Simulate realistic distance (20-300 cm) with occasional obstacles
            # This simulates a robot moving in an environment
            sim_distance = random.uniform(50.0, 250.0)
            
            # Occasionally simulate an obstacle (closer distance)
            if random.random() < 0.05:  # 5% chance
                sim_distance = random.uniform(10.0, 40.0)
            
            return sim_distance
        
        try:
            distance = self.dog.read_distance()
            
            # Validate reading
            if distance is None or distance < 0:
                return None
            
            # Filter out of range values (ultrasonic typically 2-400cm)
            if distance < 2 or distance > 400:
                return None
            
            return float(distance)
            
        except Exception as e:
            self.get_logger().debug(f"Error reading distance: {e}")
            return None
    
    def apply_moving_average(self, new_distance):
        """Apply moving average filter to smooth readings"""
        if new_distance is None:
            return None
        
        self.distance_buffer.append(new_distance)
        
        if len(self.distance_buffer) == 0:
            return None
        
        return sum(self.distance_buffer) / len(self.distance_buffer)
    
    def assess_quality(self, distance):
        """Assess the quality/reliability of distance reading"""
        if distance is None:
            return "invalid"
        
        # Check for common issues
        if distance > 300:
            return "poor"  # too far
        elif distance > 150:
            return "fair"
        elif distance < 5:
            return "poor"  # too close, may be inaccurate
        else:
            return "good"
    
    def get_quality_numeric(self, distance):
        """Get numeric quality value (0-2)"""
        quality_map = {
            "invalid": 0,
            "poor": 1,
            "fair": 1,
            "good": 2
        }
        quality = self.assess_quality(distance)
        return quality_map.get(quality, 0)
    
    def publish_distance(self):
        """Publish distance reading to ROS topics"""
        try:
            # Read raw distance
            raw_distance = self.read_distance()
            
            # Apply filtering
            filtered_distance = self.apply_moving_average(raw_distance)
            
            if filtered_distance is not None and filtered_distance > 0:
                # Publish simple Float32 message
                distance_msg = Float32()
                distance_msg.data = float(filtered_distance)
                self.distance_pub.publish(distance_msg)
                
                # Publish detailed String message with quality info
                quality = self.assess_quality(filtered_distance)
                quality_num = self.get_quality_numeric(filtered_distance)
                detailed_msg = String()
                detailed_msg.data = f"distance:{filtered_distance:.1f}cm:quality:{quality}:code:{quality_num}"
                self.distance_detailed_pub.publish(detailed_msg)
                
                # Log at debug level for troubleshooting
                self.get_logger().debug(f"Distance published: {filtered_distance:.1f} cm (quality: {quality})")
            else:
                # No valid reading, publish default values
                distance_msg = Float32()
                distance_msg.data = 999.0  # Large number indicates no reading
                self.distance_pub.publish(distance_msg)
                
                detailed_msg = String()
                detailed_msg.data = "distance:999.0cm:quality:invalid:code:0"
                self.distance_detailed_pub.publish(detailed_msg)
                
                self.get_logger().debug("No valid distance reading")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing distance: {e}")
    
    def start_head_scanning(self):
        """Start scanning head for 360-degree awareness"""
        if self.scanning:
            return
        
        self.get_logger().info("Starting head scanning for 360-degree awareness")
        self.scanning = True
        self.scan_thread = threading.Thread(target=self._head_scan_loop, daemon=True)
        self.scan_thread.start()
    
    def _head_scan_loop(self):
        """Loop to scan head left and right"""
        if self.dog is None:
            return
        
        angles = [(-45, 0, 0), (-30, 0, 0), (-15, 0, 0), (0, 0, 0),
                  (15, 0, 0), (30, 0, 0), (45, 0, 0), (30, 0, 0),
                  (15, 0, 0), (0, 0, 0), (-15, 0, 0), (-30, 0, 0)]
        
        self.get_logger().info("Head scanning active")
        
        while self.scanning and rclpy.ok():
            for yaw, pitch, roll in angles:
                try:
                    self.dog.head_move([(yaw, pitch, roll)], immediately=True, speed=50)
                    time.sleep(0.15)
                    
                    # Read and publish distance at each angle (optional)
                    dist = self.dog.read_distance()
                    if dist and dist > 0 and dist < 400:
                        self.get_logger().debug(f"Scan angle {yaw}°: {dist:.1f} cm")
                        
                except Exception as e:
                    self.get_logger().debug(f"Head scan error: {e}")
            
            time.sleep(1)  # Pause between full scans
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down distance node...")
        self.scanning = False
        
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(timeout=2.0)
        
        if self.dog:
            try:
                # Return head to center
                self.dog.head_move([(0, 0, 0)], immediately=True, speed=50)
                self.get_logger().info("Head returned to center")
            except Exception as e:
                self.get_logger().debug(f"Error returning head to center: {e}")
        
        self.get_logger().info("Distance node shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    
    node = PiDogDistanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Distance node interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
