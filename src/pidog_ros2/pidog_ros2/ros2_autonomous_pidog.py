#!/usr/bin/env python3
"""
ROS 2 Main Autonomous Node for Pi Dog
THIS IS THE ONLY NODE THAT INITIALIZES PIDOG HARDWARE
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

import threading
import time
import sys
import os

# Import PiDog Manager (only here!)
sys.path.append('/ros2_ws/src/pidog_ros2/pidog_ros2')
from pidog_manager import get_pidog_manager


class Ros2AutonomousPiDog(Node):
    def __init__(self):
        super().__init__('ros2_autonomous_pidog')
        
        # Initialize PiDog hardware (ONLY HERE!)
        self.get_logger().info("Initializing PiDog hardware...")
        self.pidog_manager = get_pidog_manager()
        
        if self.pidog_manager.initialize(disable_sensors=True):
            self.dog = self.pidog_manager.get_pidog()
            self.get_logger().info("PiDog hardware available - movement enabled")
            
            # Test movement to verify hardware
            try:
                self.get_logger().info("Testing hardware...")
                self.dog.do_action('sit', speed=50)
                self.dog.wait_all_done()
                time.sleep(1)
                self.dog.do_action('stand', speed=50)
                self.dog.wait_all_done()
                self.get_logger().info("PiDog hardware test successful")
            except Exception as e:
                self.get_logger().error(f"Hardware test failed: {e}")
                return
        else:
            self.dog = None
            self.get_logger().error("PiDog hardware NOT available - exiting")
            return
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers (for other nodes to subscribe to)
        self.distance_pub = self.create_publisher(Float32, 'distance', qos_profile)
        self.imu_pub = self.create_publisher(String, 'imu', qos_profile)
        self.touch_pub = self.create_publisher(String, 'touch', qos_profile)
        self.sound_direction_pub = self.create_publisher(String, 'sound_direction', qos_profile)
        self.status_pub = self.create_publisher(String, 'status', qos_profile)
        
        # Subscriber for movement commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        self.command_sub = self.create_subscription(
            String, 'command',
            self.command_callback,
            qos_profile
        )
        
        # Start threads
        self.sensor_thread = threading.Thread(target=self.sensor_reading_loop, daemon=True)
        self.sensor_thread.start()
        
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("ROS 2 Autonomous Pi Dog Node Ready")
        self.get_logger().info("Publishing: distance, imu, touch, sound_direction")
        self.get_logger().info("Subscribing to: cmd_vel, command")
        
        # Publish initial status
        self.status_pub.publish(String(data="node:started"))
    
    def sensor_reading_loop(self):
        """Read sensors and publish data"""
        self.get_logger().info("Starting sensor reading loop")
        
        # Give hardware time to initialize
        time.sleep(2)
        
        loop_count = 0
        while rclpy.ok() and self.dog is not None:
            try:
                loop_count += 1
                
                # Publish heartbeat every 50 loops
                if loop_count % 50 == 0:
                    self.get_logger().info(f"Sensor loop active (count: {loop_count})")
                    self.status_pub.publish(String(data=f"heartbeat:{loop_count}"))
                
                # Read distance sensor
                try:
                    if hasattr(self.dog, 'ultrasonic'):
                        distance = self.dog.ultrasonic.read_distance()
                        if distance and 2 <= distance <= 400:
                            self.distance_pub.publish(Float32(data=float(distance)))
                            if loop_count % 20 == 0:  # Log every 2 seconds
                                self.get_logger().info(f"Distance: {distance:.1f} cm")
                except Exception as e:
                    self.get_logger().debug(f"Distance read error: {e}")
                
                # Read touch sensors
                try:
                    if hasattr(self.dog, 'dual_touch'):
                        for i in range(4):
                            if hasattr(self.dog.dual_touch, 'read'):
                                touched = self.dog.dual_touch.read(i)
                                if touched:
                                    self.touch_pub.publish(String(data=f"touched:{i}"))
                                    self.get_logger().info(f"Touch detected on sensor {i}")
                except Exception as e:
                    self.get_logger().debug(f"Touch read error: {e}")
                
                # Read sound direction
                try:
                    if hasattr(self.dog, 'ears'):
                        if hasattr(self.dog.ears, 'isdetected') and self.dog.ears.isdetected():
                            if hasattr(self.dog.ears, 'read'):
                                angle = self.dog.ears.read()
                                if angle is not None and angle >= 0:
                                    self.sound_direction_pub.publish(String(data=f"{angle:.1f}:detected"))
                                    if loop_count % 20 == 0:
                                        self.get_logger().info(f"Sound direction: {angle:.1f}°")
                except Exception as e:
                    self.get_logger().debug(f"Sound direction error: {e}")
                
                # For IMU, publish a simple test message if no real data
                # This ensures the topic is active
                if loop_count % 10 == 0:  # Every 1 second
                    test_imu = String()
                    test_imu.data = f"0.0,0.0,{loop_count % 360}"
                    self.imu_pub.publish(test_imu)
                
                time.sleep(0.1)  # 10Hz reading
                
            except Exception as e:
                self.get_logger().error(f"Sensor loop error: {e}")
                time.sleep(1.0)
        
        self.get_logger().warning("Sensor reading loop ended")
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        if linear > 0:
            self.execute_movement('forward', step_count=int(linear * 10), speed=70)
        elif linear < 0:
            self.execute_movement('backward', step_count=int(abs(linear) * 10), speed=70)
        elif angular > 0:
            self.execute_movement('turn_right', step_count=int(angular * 10), speed=70)
        elif angular < 0:
            self.execute_movement('turn_left', step_count=int(abs(angular) * 10), speed=70)
    
    def command_callback(self, msg: String):
        """Handle command strings"""
        self.get_logger().info(f"Command received: {msg.data}")
        parts = msg.data.lower().split(':')
        command = parts[0]
        step_count = int(parts[1]) if len(parts) > 1 else 5
        speed = int(parts[2]) if len(parts) > 2 else 70
        self.execute_movement(command, step_count, speed)
    
    def execute_movement(self, command, step_count=5, speed=70):
        """Execute movement on hardware"""
        if self.dog is None:
            self.get_logger().warn(f"Cannot execute {command} - no hardware")
            return
        
        try:
            self.get_logger().info(f"Executing: {command} (steps={step_count}, speed={speed})")
            
            if command == 'stop':
                self.dog.body_stop()
                self.dog.wait_all_done()
            elif command in ['sit', 'stand']:
                self.dog.do_action(command, speed=speed)
                self.dog.wait_all_done()
            else:
                self.dog.do_action(command, step_count=step_count, speed=speed)
                self.dog.wait_all_done()
                
        except Exception as e:
            self.get_logger().error(f"Movement error: {e}")
    
    def publish_status(self):
        """Publish node status"""
        status_msg = String()
        status_msg.data = f"state:running:hardware:{self.dog is not None}"
        self.status_pub.publish(status_msg)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down...")
        if self.dog:
            try:
                self.dog.do_action('sit', speed=50)
                self.dog.wait_all_done()
                self.pidog_manager.shutdown()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    
    node = Ros2AutonomousPiDog()
    
    if node.dog is None:
        node.get_logger().error("No hardware - exiting")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
