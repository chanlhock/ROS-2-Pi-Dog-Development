#!/usr/bin/env python3
"""
ROS 2 Sound Direction Sensor Node for Pi Dog
NO HARDWARE ACCESS - subscribes to sound direction data from main node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from std_srvs.srv import SetBool

import time


class PiDogSoundDirectionNode(Node):
    """
    ROS 2 Node for PiDog sound direction sensor.
    
    This node does NOT access hardware directly. It receives sound direction
    data from the main autonomous node via the 'sound_direction' topic.
    """
    
    def __init__(self):
        super().__init__('pidog_direction_sensor_node')
        
        self.get_logger().info("Sound direction node starting - listening for data from main node")
        
        # Parameters
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        
        # State variables
        self.current_angle = -1.0
        self.current_direction = "unknown"
        self.sound_detected = False
        self.last_detection_time = time.time()
        
        # Service to enable/disable
        self.enable_srv = self.create_service(SetBool, 'enable_sound_direction', self.enable_callback)
        
        # Publisher for processed sound direction
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.direction_pub = self.create_publisher(String, 'sound_direction_processed', qos_profile)
        
        # Subscriber to main node's sound direction data
        self.sound_sub = self.create_subscription(
            String,
            'sound_direction',
            self.sound_callback,
            qos_profile
        )
        
        self.get_logger().info("PiDog Sound Direction Node Ready")
    
    def enable_callback(self, request, response):
        """Enable/disable sound direction processing."""
        self.enabled = request.data
        response.success = True
        response.message = f"Sound direction {'enabled' if self.enabled else 'disabled'}"
        self.get_logger().info(f"Sound direction: {'ON' if self.enabled else 'OFF'}")
        return response
    
    def angle_to_direction(self, angle: float) -> str:
        """Convert angle to human-readable direction."""
        if angle < 0:
            return "unknown"
        if angle < 45 or angle >= 315:
            return "front"
        elif 45 <= angle < 135:
            return "right"
        elif 135 <= angle < 225:
            return "back"
        else:
            return "left"
    
    def sound_callback(self, msg: String):
        """Receive sound direction from main node."""
        if not self.enabled:
            return
        
        try:
            # Parse message format "angle:detected" or "angle:direction:detected"
            parts = msg.data.split(':')
            if len(parts) >= 1:
                self.current_angle = float(parts[0])
                self.current_direction = self.angle_to_direction(self.current_angle)
                self.sound_detected = len(parts) > 1 and parts[-1] == "detected"
                self.last_detection_time = time.time()
                
                # Publish processed direction
                processed_msg = String()
                processed_msg.data = f"{self.current_angle:.1f}:{self.current_direction}:{1 if self.sound_detected else 0}"
                self.direction_pub.publish(processed_msg)
                
                if self.sound_detected:
                    self.get_logger().info(f"🔊 Sound from {self.current_direction} ({self.current_angle:.0f}°)")
                    
        except Exception as e:
            self.get_logger().debug(f"Sound direction parse error: {e}")
    
    def shutdown(self):
        self.get_logger().info("Sound direction node shutting down")


def main(args=None):
    rclpy.init(args=args)
    node = PiDogSoundDirectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Sound direction node interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
