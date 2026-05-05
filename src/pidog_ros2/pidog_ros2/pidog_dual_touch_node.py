#!/usr/bin/env python3
"""
ROS 2 Dual Touch Sensor Node for Pi Dog
Detects when touch sensors are activated
Uses standard ROS 2 message types (no custom imports required)
"""

import rclpy
from rclpy.node import Node

# Standard ROS 2 imports instead of custom ones
from std_msgs.msg import String

import time
import threading
import random


class PiDogDualTouchNode(Node):
    def __init__(self):
        super().__init__('pidog_dual_touch_node')
        
        # Parameters
        self.declare_parameter('touch_debounce_time', 0.1)  # seconds
        self.declare_parameter('simulate_touch', True)  # Simulate if no hardware
        self.declare_parameter('touch_probability', 0.02)  # 2% chance per poll in sim
        
        self.debounce_time = self.get_parameter('touch_debounce_time').value
        self.simulate_touch = self.get_parameter('simulate_touch').value
        self.touch_probability = self.get_parameter('touch_probability').value
        
        self.dog = None
        
        # Publisher (using String for touch events)
        self.touch_pub = self.create_publisher(String, 'touch', 10)
        
        # State tracking
        self.last_touch_time = {
            'head': 0,
            'back': 0,
            'left': 0,
            'right': 0
        }
        
        # Touch active state
        self.touch_active = {
            'head': False,
            'back': False,
            'left': False,
            'right': False
        }
        
        # Try to initialize PiDog hardware
        if not self.simulate_touch:
            try:
                from pidog import Pidog
                self.dog = Pidog()
                self.get_logger().info("PiDog hardware initialized for touch sensing")
                self.simulate_touch = False
            except ImportError:
                self.get_logger().warning("PiDog library not available - running in simulation mode")
                self.simulate_touch = True
            except Exception as e:
                self.get_logger().error(f"Failed to initialize PiDog: {e}")
                self.simulate_touch = True
        
        # Timer for touch polling (20 Hz)
        self.create_timer(0.05, self.check_touch)
        
        self.get_logger().info("PiDog Dual Touch Node started")
        self.get_logger().info(f"Simulation mode: {self.simulate_touch}")
        self.get_logger().info("Publishing touch events to /touch topic")
        
        if self.simulate_touch:
            self.get_logger().info(f"Touch probability: {self.touch_probability * 100}% per poll")
    
    def read_hardware_touch(self):
        """Read actual touch sensor from PiDog hardware"""
        if self.dog is None:
            return None
        
        try:
            # Try to read touch sensors from PiDog
            # Method names may vary - adjust based on actual PiDog API
            touches = {}
            
            # Check head touch (if available)
            if hasattr(self.dog, 'read_head_touch'):
                touches['head'] = self.dog.read_head_touch()
            
            # Check back touch (if available)
            if hasattr(self.dog, 'read_back_touch'):
                touches['back'] = self.dog.read_back_touch()
            
            # Check left/right sensors if available
            if hasattr(self.dog, 'read_touch_sensors'):
                sensors = self.dog.read_touch_sensors()
                touches.update(sensors)
            
            return touches
            
        except Exception as e:
            self.get_logger().debug(f"Error reading touch sensors: {e}")
            return None
    
    def format_touch_message(self, location, touched, confidence=1.0):
        """Format touch event as String message
        Format: "touched:{location}:{confidence}"
        Or: "released:{location}"
        """
        if touched:
            return f"touched:{location}:{confidence:.2f}"
        else:
            return f"released:{location}"
    
    def publish_touch(self, location, confidence=1.0):
        """Publish touch event"""
        current_time = time.time()
        
        # Debounce check
        if current_time - self.last_touch_time[location] < self.debounce_time:
            return
        
        # Check if already active to avoid duplicate events
        if self.touch_active[location]:
            return
        
        self.last_touch_time[location] = current_time
        self.touch_active[location] = True
        
        # Publish touched message
        msg = String()
        msg.data = self.format_touch_message(location, True, confidence)
        self.touch_pub.publish(msg)
        self.get_logger().info(f"👆 Touch detected at {location} (confidence: {confidence:.2f})")
        
        # Schedule release after a short time
        def clear_touch():
            time.sleep(0.5)
            if self.touch_active[location]:
                self.touch_active[location] = False
                msg_clear = String()
                msg_clear.data = self.format_touch_message(location, False)
                self.touch_pub.publish(msg_clear)
                self.get_logger().debug(f"Touch released at {location}")
        
        threading.Thread(target=clear_touch, daemon=True).start()
    
    def simulate_touch_check(self):
        """Simulate random touch events for testing"""
        # Random chance of touch per sensor
        for location in ['head', 'back', 'left', 'right']:
            if not self.touch_active[location] and random.random() < self.touch_probability:
                confidence = random.uniform(0.7, 0.99)
                self.publish_touch(location, confidence)
    
    def hardware_touch_check(self):
        """Check actual hardware touch sensors"""
        touches = self.read_hardware_touch()
        
        if touches is None:
            # Fallback to simulation if hardware read fails
            self.simulate_touch_check()
            return
        
        # Process each touch sensor
        for location, touched in touches.items():
            if touched and not self.touch_active[location]:
                # New touch detected
                confidence = 0.95  # Default confidence for hardware
                self.publish_touch(location, confidence)
            elif not touched and self.touch_active[location]:
                # Touch released
                self.touch_active[location] = False
                msg = String()
                msg.data = self.format_touch_message(location, False)
                self.touch_pub.publish(msg)
                self.get_logger().debug(f"Touch released at {location}")
    
    def check_touch(self):
        """Check touch sensor status"""
        if self.simulate_touch:
            self.simulate_touch_check()
        else:
            self.hardware_touch_check()
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down touch node...")
        
        # Publish release messages for all active touches
        for location in self.touch_active:
            if self.touch_active[location]:
                msg = String()
                msg.data = self.format_touch_message(location, False)
                self.touch_pub.publish(msg)
        
        if self.dog:
            try:
                self.dog.close()
                self.get_logger().info("PiDog hardware closed")
            except Exception as e:
                self.get_logger().debug(f"Error closing PiDog: {e}")
        
        self.get_logger().info("Touch node shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    
    node = PiDogDualTouchNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Touch node interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():  # Check if already shutdown
            rclpy.shutdown()


if __name__ == '__main__':
    main()
