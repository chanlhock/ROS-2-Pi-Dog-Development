#!/usr/bin/env python3
"""
##########################################################################
# ROS 2 Autonomous Sunfounder Pi Dog with Raspberry Pi 5 in Ubuntu 22.04
#
# ROS 2 IMU Node for Pi Dog
# Receives IMU data from main node and republishes in standard formats
#  
# Copyright (c) 2026 Bernard Chan
# chanlhock@gmail.com
#
# Date           Author          Notes
# 05/05/2026     Bernard Chan    Initial release
#
# pidog_imu_node.py is licensed under the GNU General Public License v3.0
# License v3.0 Permissions of this strong copyleft license are 
# conditioned on making available complete source code of licensed 
# works and modifications, which include larger works using a licensed 
# work, under the same license. Copyright and license notices must be 
# preserved. Contributors provide an express grant of patent rights.
##########################################################################
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import String

import math
import time


class PiDogIMUNode(Node):
    def __init__(self):
        super().__init__('pidog_imu_node')
        
        self.get_logger().info("IMU node starting (NO hardware initialization)")
        
        # Parameters
        self.declare_parameter('publish_frequency', 10.0)
        self.publish_freq = self.get_parameter('publish_frequency').value
        
        # Publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )
        
        self.standard_imu_pub = self.create_publisher(Imu, 'imu/raw', qos_profile)
        self.simple_imu_pub = self.create_publisher(String, 'imu/simple', qos_profile)
        
        # Subscriber to main node's IMU data
        self.imu_sub = self.create_subscription(
            String,
            'imu',
            self.imu_callback,
            qos_profile
        )
        
        # Timer for publishing processed data
        timer_period = 1.0 / self.publish_freq
        self.timer = self.create_timer(timer_period, self.publish_imu)
        
        # Current IMU data storage
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 9.81  # Default gravity
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        
        self.get_logger().info(f"IMU node ready (publishing at {self.publish_freq} Hz)")
    
    def imu_callback(self, msg: String):
        """Receive IMU data from main node."""
        try:
            # Format: roll,pitch,yaw,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
            parts = msg.data.split(',')
            if len(parts) >= 9:
                self.roll = float(parts[0])
                self.pitch = float(parts[1])
                self.yaw = float(parts[2])
                self.accel_x = float(parts[3])
                self.accel_y = float(parts[4])
                self.accel_z = float(parts[5])
                self.gyro_x = float(parts[6])
                self.gyro_y = float(parts[7])
                self.gyro_z = float(parts[8])
            elif len(parts) >= 3:
                # Fallback for old format (only orientation)
                self.roll = float(parts[0])
                self.pitch = float(parts[1])
                self.yaw = float(parts[2])
        except Exception as e:
            self.get_logger().debug(f"IMU parse error: {e}")
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        
        return qx, qy, qz, qw
    
    def publish_imu(self):
        """Publish IMU data in standard ROS2 format."""
        
        # Create standard IMU message (sensor_msgs/Imu)
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Set orientation from Euler angles
        qx, qy, qz, qw = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        # Set angular velocity (convert deg/s to rad/s)
        imu_msg.angular_velocity.x = self.gyro_x * (math.pi / 180.0)
        imu_msg.angular_velocity.y = self.gyro_y * (math.pi / 180.0)
        imu_msg.angular_velocity.z = self.gyro_z * (math.pi / 180.0)
        imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        # Set linear acceleration (convert g to m/s^2)
        imu_msg.linear_acceleration.x = self.accel_x * 9.81
        imu_msg.linear_acceleration.y = self.accel_y * 9.81
        imu_msg.linear_acceleration.z = self.accel_z * 9.81
        imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        self.standard_imu_pub.publish(imu_msg)
        
        # Publish simple string message for easy viewing
        simple_msg = String()
        simple_msg.data = f"r:{self.roll:.1f},p:{self.pitch:.1f},y:{self.yaw:.1f}"
        self.simple_imu_pub.publish(simple_msg)
        
        # Log occasionally for debugging (every 50 publishes ~5 seconds)
        if int(time.time()) % 5 == 0 and int(time.time() * 10) % 50 == 0:
            self.get_logger().debug(f"IMU: roll={self.roll:.1f}°, pitch={self.pitch:.1f}°, yaw={self.yaw:.1f}°")
    
    def shutdown(self):
        self.get_logger().info("IMU node shutting down")


def main(args=None):
    rclpy.init(args=args)
    node = PiDogIMUNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("IMU node interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()