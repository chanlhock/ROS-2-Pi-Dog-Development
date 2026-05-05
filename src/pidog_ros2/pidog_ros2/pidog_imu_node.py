#!/usr/bin/env python3
"""
ROS 2 IMU Node for Pi Dog
Publishes IMU data for orientation and motion sensing
Uses standard ROS 2 message types (no custom imports required)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Standard ROS 2 imports
from sensor_msgs.msg import Imu
from std_msgs.msg import String

import math
import time
import random


class PiDogIMUNode(Node):
    def __init__(self):
        super().__init__('pidog_imu_node')
        
        # Parameters
        self.declare_parameter('publish_frequency', 20.0)  # Hz
        self.declare_parameter('simulate_imu', True)  # Simulate if no hardware
        self.declare_parameter('use_hardware_imu', False)
        
        self.publish_freq = self.get_parameter('publish_frequency').value
        self.simulate = self.get_parameter('simulate_imu').value
        self.use_hardware = self.get_parameter('use_hardware_imu').value
        
        # IMU data
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.accel = [0.0, 0.0, 9.81]
        self.gyro = [0.0, 0.0, 0.0]
        
        # For simulation
        self.sim_time = 0.0
        self.last_sim_update = time.time()
        
        # ROS 2 Publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )
        
        # Standard IMU publisher (sensor_msgs/Imu)
        self.standard_imu_pub = self.create_publisher(
            Imu,
            'imu/raw',
            qos_profile
        )
        
        # String publisher for simplified IMU data
        self.simple_imu_pub = self.create_publisher(
            String,
            'imu/simple',
            qos_profile
        )
        
        # Initialize hardware IMU if available
        self.imu_device = None
        if self.use_hardware:
            self.init_hardware_imu()
        
        # Start publishing timer
        timer_period = 1.0 / self.publish_freq
        self.timer = self.create_timer(timer_period, self.publish_imu_data)
        
        self.get_logger().info(f"PiDog IMU Node Ready (simulate={self.simulate}, hardware={self.use_hardware})")
        self.get_logger().info(f"Publishing to: imu/raw and imu/simple")
    
    def init_hardware_imu(self):
        """Initialize hardware IMU (e.g., MPU6050)"""
        try:
            import board
            import busio
            import adafruit_mpu6050
            
            i2c = busio.I2C(board.SCL, board.SDA)
            self.imu_device = adafruit_mpu6050.MPU6050(i2c)
            self.get_logger().info("Hardware IMU initialized (MPU6050)")
            self.use_hardware = True
            self.simulate = False
            
        except ImportError:
            self.get_logger().warning("Adafruit IMU library not available, using simulation")
            self.use_hardware = False
            self.simulate = True
        except Exception as e:
            self.get_logger().warning(f"Failed to initialize hardware IMU: {e}")
            self.use_hardware = False
            self.simulate = True
    
    def read_hardware_imu(self):
        """Read data from hardware IMU"""
        if self.imu_device is None:
            return None
        
        try:
            accel = self.imu_device.acceleration
            gyro = self.imu_device.gyro
            return {'accel': accel, 'gyro': gyro}
        except Exception as e:
            self.get_logger().debug(f"IMU read error: {e}")
            return None
    
    def simulate_imu_update(self):
        """Generate simulated IMU data for testing"""
        current_time = time.time()
        dt = min(current_time - self.last_sim_update, 0.1)
        self.last_sim_update = current_time
        
        # Simple simulation with some noise and oscillation
        self.sim_time += dt
        
        # Simulate slight movements
        self.roll = 5.0 * math.sin(self.sim_time * 0.5) + random.uniform(-0.3, 0.3)
        self.pitch = 3.0 * math.sin(self.sim_time * 0.7 + 1) + random.uniform(-0.3, 0.3)
        self.yaw += (0.5 * math.sin(self.sim_time * 0.2) + random.uniform(-0.2, 0.2)) * dt
        
        # Normalize yaw to 0-360
        self.yaw = self.yaw % 360.0
        
        # Simulate accelerometer (gravity + movement)
        self.accel[0] = 0.1 * math.sin(self.sim_time) + random.uniform(-0.05, 0.05)
        self.accel[1] = 0.1 * math.cos(self.sim_time * 1.3) + random.uniform(-0.05, 0.05)
        self.accel[2] = 9.81 + 0.2 * math.sin(self.sim_time * 2) + random.uniform(-0.1, 0.1)
        
        # Simulate gyroscope (convert to rad/s for ROS standard)
        self.gyro[0] = 0.5 * math.cos(self.sim_time) + random.uniform(-0.02, 0.02)
        self.gyro[1] = 0.3 * math.sin(self.sim_time * 1.5) + random.uniform(-0.02, 0.02)
        self.gyro[2] = 0.2 * math.cos(self.sim_time * 0.8) + random.uniform(-0.02, 0.02)
    
    def update_real_imu(self):
        """Update from real IMU hardware"""
        data = self.read_hardware_imu()
        if data:
            ax, ay, az = data['accel']
            
            # Calculate roll and pitch from accelerometer
            self.roll = math.atan2(ay, az) * 180.0 / math.pi
            self.pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180.0 / math.pi
            
            # Gyroscope integration for yaw
            gx, gy, gz = data['gyro']
            dt = 1.0 / self.publish_freq
            self.yaw += gz * dt * (180.0 / math.pi)
            self.yaw = self.yaw % 360.0
            
            self.accel = list(data['accel'])
            self.gyro = list(data['gyro'])
    
    def format_simple_imu_message(self):
        """Format IMU data as simple comma-separated string"""
        return f"{self.roll:.2f},{self.pitch:.2f},{self.yaw:.2f}," \
               f"{self.accel[0]:.3f},{self.accel[1]:.3f},{self.accel[2]:.3f}," \
               f"{self.gyro[0]:.6f},{self.gyro[1]:.6f},{self.gyro[2]:.6f}"
    
    def publish_imu_data(self):
        """Publish IMU data to ROS topics"""
        # Update data source
        if self.use_hardware:
            self.update_real_imu()
        elif self.simulate:
            self.simulate_imu_update()
        
        # Create and publish standard IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Convert Euler angles (roll, pitch, yaw) to quaternion
        # Using standard conversion: q = [x, y, z, w]
        cy = math.cos(math.radians(self.yaw * 0.5))
        sy = math.sin(math.radians(self.yaw * 0.5))
        cp = math.cos(math.radians(self.pitch * 0.5))
        sp = math.sin(math.radians(self.pitch * 0.5))
        cr = math.cos(math.radians(self.roll * 0.5))
        sr = math.sin(math.radians(self.roll * 0.5))
        
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        
        # Set orientation covariance (identity matrix as 9-element array)
        # Format: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        imu_msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = self.gyro[0]
        imu_msg.angular_velocity.y = self.gyro[1]
        imu_msg.angular_velocity.z = self.gyro[2]
        imu_msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        
        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = self.accel[0]
        imu_msg.linear_acceleration.y = self.accel[1]
        imu_msg.linear_acceleration.z = self.accel[2]
        imu_msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        self.standard_imu_pub.publish(imu_msg)
        
        # Publish simple string message
        simple_msg = String()
        simple_msg.data = self.format_simple_imu_message()
        self.simple_imu_pub.publish(simple_msg)
        
        # Log occasionally
        if int(time.time()) % 10 == 0 and self.simulate:
            self.get_logger().info(
                f"IMU: roll={self.roll:.1f}°, pitch={self.pitch:.1f}°, yaw={self.yaw:.1f}°"
            )
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("IMU node shutting down...")


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
        if rclpy.ok():  # Check if already shutdown
            rclpy.shutdown()


if __name__ == '__main__':
    main()
