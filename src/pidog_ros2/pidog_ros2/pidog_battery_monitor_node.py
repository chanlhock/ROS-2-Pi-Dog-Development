#!/usr/bin/env python3
"""
##########################################################################
# ROS 2 Battery Monitor Node for Pi Dog with Raspberry Pi 5
#
# Monitors battery voltage from robot_hat ADC and provides:
# - Periodic battery status publication
# - On-demand battery voltage queries via service
# - Battery level warnings and alerts
#
# Copyright (c) 2026 Bernard Chan
# chanlhock@gmail.com
#
# Date           Author          Notes
# 13/05/2026     Bernard Chan    Initial release
#
##########################################################################
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Float32, String

import threading
import time
from robot_hat import ADC

# Battery configuration
BATTERY_ADC_CHANNEL = "A4"  # ADC channel for battery voltage
BATTERY_CHECK_INTERVAL = 60  # Check battery every 60 seconds
BATTERY_WARNING_THRESHOLD = 7.0  # Warn if below 7.0V
BATTERY_CRITICAL_THRESHOLD = 6.5  # Critical if below 6.5V
BATTERY_MAX_VOLTAGE = 8.4  # Maximum battery voltage (8.4V nominal)
BATTERY_MIN_VOLTAGE = 6.0  # Minimum safe voltage


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Initializing Battery Monitor Node...")
        
        # ============================================================
        # INITIALIZE BATTERY ADC
        # ============================================================
        self.battery_adc = None
        self.current_voltage = 0.0
        self.battery_status = "unknown"
        self.last_warning_time = 0
        
        try:
            self.battery_adc = ADC(BATTERY_ADC_CHANNEL)
            # Test read to ensure ADC is working
            test_voltage = self.battery_adc.read_voltage() * 3  # Assuming voltage divider with 3:1 ratio
            self.get_logger().info(f"✓ Battery ADC initialized on channel {BATTERY_ADC_CHANNEL}")
            self.get_logger().info(f"✓ Initial battery voltage: {test_voltage:.2f}V")
            self.current_voltage = test_voltage
        except Exception as e:
            self.get_logger().error(f"✗ Failed to initialize battery ADC: {e}")
            self.battery_adc = None
            return
        
        # ============================================================
        # ROS 2 PUBLISHERS
        # ============================================================
        qos_best = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_rel = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)
        
        # Publisher for battery voltage (continuous)
        self.battery_voltage_pub = self.create_publisher(
            Float32, 'battery_voltage', qos_best
        )
        
        # Publisher for battery status (continuous)
        self.battery_status_pub = self.create_publisher(
            String, 'battery_status', qos_rel
        )
        
        # Publisher for battery alerts
        self.battery_alert_pub = self.create_publisher(
            String, 'battery_alert', qos_rel
        )
        
        # ============================================================
        # ROS 2 SERVICES
        # ============================================================
        # Service to get current battery level on-demand
        #self.battery_service = self.create_service(
        #    GetBatteryLevel,
        #    'get_battery_level',
        #    self.get_battery_level_callback
        #)
        
        # ============================================================
        # START MONITORING THREAD
        # ============================================================
        self.monitor_thread = threading.Thread(
            target=self.battery_monitoring_loop,
            daemon=True
        )
        self.monitor_thread.start()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Battery Monitor Node Ready!")
        self.get_logger().info(f"Battery check interval: {BATTERY_CHECK_INTERVAL}s")
        self.get_logger().info(f"Warning threshold: {BATTERY_WARNING_THRESHOLD}V")
        self.get_logger().info(f"Critical threshold: {BATTERY_CRITICAL_THRESHOLD}V")
        self.get_logger().info("=" * 60)
    
    def battery_monitoring_loop(self):
        """Periodic battery monitoring loop - runs every 60 seconds by default"""
        self.get_logger().info("Battery monitoring thread started")
        time.sleep(2)  # Initial delay
        
        while rclpy.ok() and self.battery_adc:
            try:
                # Read battery voltage
                voltage = self.battery_adc.read_voltage() * 3  # Assuming voltage divider with 3:1 ratio
                
                if voltage is None or voltage == 0.0:
                    self.get_logger().debug("Skipping invalid battery reading")
                    time.sleep(1)
                    continue

                #if voltage and BATTERY_MIN_VOLTAGE <= voltage <= BATTERY_MAX_VOLTAGE:
                if BATTERY_MIN_VOLTAGE <= voltage <= BATTERY_MAX_VOLTAGE:
                    self.current_voltage = voltage
                    
                    # Calculate battery percentage (0-100%)
                    battery_percent = self.calculate_battery_percentage(voltage)
                    
                    # Determine battery status
                    if voltage >= BATTERY_WARNING_THRESHOLD:
                        self.battery_status = "good"
                    elif voltage >= BATTERY_CRITICAL_THRESHOLD:
                        self.battery_status = "warning"
                    else:
                        self.battery_status = "critical"
                    
                    # Publish voltage
                    self.battery_voltage_pub.publish(Float32(data=voltage))
                    
                    # Publish status
                    status_msg = String()
                    status_msg.data = f"voltage:{voltage:.2f}:percentage:{battery_percent:.1f}:status:{self.battery_status}"
                    self.battery_status_pub.publish(status_msg)
                    
                    # Log battery info
                    self.get_logger().info(
                        f"🔋 Battery: {voltage:.2f}V ({battery_percent:.1f}%) - {self.battery_status.upper()}"
                    )
                    
                    # Publish alerts if needed
                    if self.battery_status == "warning":
                        current_time = time.time()
                        # Only alert every 5 minutes to avoid spam
                        if current_time - self.last_warning_time > 300:
                            alert_msg = String()
                            alert_msg.data = f"warning:battery_low:{voltage:.2f}V"
                            self.battery_alert_pub.publish(alert_msg)
                            self.get_logger().warning(f"⚠️ BATTERY WARNING: {voltage:.2f}V - Please charge soon!")
                            self.last_warning_time = current_time
                    
                    elif self.battery_status == "critical":
                        alert_msg = String()
                        alert_msg.data = f"critical:battery_critical:{voltage:.2f}V"
                        self.battery_alert_pub.publish(alert_msg)
                        self.get_logger().error(f"🔴 CRITICAL BATTERY: {voltage:.2f}V - CHARGE IMMEDIATELY!")
                    
                else:
                    self.get_logger().warning(f"Invalid battery reading: {voltage}V")
                    self.battery_status = "unknown"
                
                # Wait for the configured interval (default 60 seconds)
                time.sleep(BATTERY_CHECK_INTERVAL)
                
            except Exception as e:
                self.get_logger().error(f"Battery read error: {e}")
                self.battery_status = "error"
                time.sleep(10)  # Retry after 10 seconds if error
    
    def calculate_battery_percentage(self, voltage):
        """
        Calculate battery percentage from voltage
        For 12V LiPo batteries (2S):
        - 8.4V = 100%
        - 7.2V = 50% 
        - 6.0V = 0% (cutoff)
        
        Adjust these values based on your actual battery type
        """
        # LiPo 2S battery curve (approximate)
        voltage_max = 8.4  # 4.2V per cell * 2 cells
        voltage_min = 6.0  # 3.0V per cell * 2 cells (cutoff)
        
        if voltage >= voltage_max:
            return 100.0
        elif voltage <= voltage_min:
            return 0.0
        else:
            # Linear approximation (can be improved with actual curve)
            percentage = ((voltage - voltage_min) / (voltage_max - voltage_min)) * 100.0
            return max(0.0, min(100.0, percentage))
    
    def get_battery_level_callback(self, request, response):
        """
        Service callback to get battery level on-demand
        
        Service format:
        Request: empty
        Response: voltage (float), percentage (float), status (string)
        """
        try:
            # Read current battery voltage
            if self.battery_adc:
                voltage = self.battery_adc.read_voltage()
                if voltage and BATTERY_MIN_VOLTAGE <= voltage <= BATTERY_MAX_VOLTAGE:
                    self.current_voltage = voltage
                else:
                    voltage = self.current_voltage
            else:
                voltage = self.current_voltage
            
            percentage = self.calculate_battery_percentage(voltage)
            
            # Determine status
            if voltage >= BATTERY_WARNING_THRESHOLD:
                status = "good"
            elif voltage >= BATTERY_CRITICAL_THRESHOLD:
                status = "warning"
            else:
                status = "critical"
            
            # Populate response
            response.voltage = voltage
            response.percentage = percentage
            response.status = status
            
            self.get_logger().info(
                f"📊 Battery level requested: {voltage:.2f}V ({percentage:.1f}%) - {status}"
            )
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in battery service: {e}")
            response.voltage = self.current_voltage
            response.percentage = self.calculate_battery_percentage(self.current_voltage)
            response.status = "error"
            return response
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down Battery Monitor Node...")
        if self.battery_adc:
            try:
                # ADC cleanup if needed
                pass
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    
    node = BatteryMonitorNode()
    
    if node.battery_adc is None:
        node.get_logger().error("No battery ADC available - exiting")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down battery monitor node")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
