#!/usr/bin/env python3
"""
ROS 2 IMU Node for Pi Dog
NO HARDWARE ACCESS - receives data from main node
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
        
        # Current IMU data
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.accel = [0.0, 0.0, 9.81]
        self.gyro = [0.0, 0.0, 0.0]
        
        self.get_logger().info(f"IMU node ready (publishing at {self.publish_freq} Hz)")
    
    def imu_callback(self, msg: String):
        """Receive IMU data from main node."""
        try:
            parts = msg.data.split(',')
            if len(parts) >= 3:
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
        """Publish IMU data."""
        # Create standard IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        qx, qy, qz, qw = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        
        imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        self.standard_imu_pub.publish(imu_msg)
        
        # Publish simple message
        simple_msg = String()
        simple_msg.data = f"{self.roll:.2f},{self.pitch:.2f},{self.yaw:.2f}"
        self.simple_imu_pub.publish(simple_msg)
    
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
