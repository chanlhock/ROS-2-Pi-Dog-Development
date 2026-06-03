#!/usr/bin/env python3
"""
##########################################################################
# ROS 2 Autonomous Sunfounder Pi Dog with Raspberry Pi 5 in Ubuntu 22.04
#
# ROS 2 Main Autonomous Node for Pi Dog
# THIS IS THE ONLY NODE THAT INITIALIZES PIDOG HARDWARE
# Features: Autonomous Wandering | Obstacle Avoidance | Voice Commands | 
#           Emotions | Personality Actions
#  
# Copyright (c) 2026 Bernard Chan
# chanlhock@gmail.com
#
# Date           Author          Notes
# 05/05/2026     Bernard Chan    Initial release
# 08/05/2026     Bernard Chan    This version runs on Docker with
#                                Ubuntu 22.04 and ROS 2 Humble
# 15/05/2026     Bernard Chan    Added battery monitoring 
# 24/05/2026     Bernard Chan    Added picked up detection using IMU raw data
#
# ros2_autonomous_pidog.py is licensed under the GNU General Public 
# License v3.0 Permissions of this strong copyleft license are 
# conditioned on making available complete source code of licensed 
# works and modifications, which include larger works using a licensed 
# work, under the same license. Copyright and license notices must be 
# preserved. Contributors provide an express grant of patent rights.
##########################################################################
"""

from flask import logging
from numpy import roll
import pidog

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from pidog.preset_actions import *
import pygame

import threading
import time
import random
import math
from collections import deque
from enum import Enum

# Import PiDog Manager
from .pidog_manager import get_pidog_manager

# Constants
# Pi Dog's name
NAME = "Woofer" # Name of the dog during my childhood
GREETING_EN = f"Hi, I am {NAME}. Your obedient ROS2 Pi Dog"
SOUNDS_PATH = "/pidog/sounds/"
OBSTACLE_DISTANCE_CM = 35  # Increased from 30cm to 35cm for better obstacle avoidance
FORWARD_SPEED = 100  # Was 80 very slow - range is 0-100
TURN_STEPS = 8  # New constant for turn steps
TURN_SPEED = 100      # Increase from 85 to 95 (was 70)
BACKWARD_SPEED = 100   # New constant for backing up
BACKWARD_TIME = 0.2 # Reduced from 1.0
TURN_TIME = 0.1      # Reduced from 0.6

class RobotState(Enum):
    IDLE = "idle"
    WANDERING = "wandering"
    AVOIDING = "avoiding"
    FOLLOWING = "following"
    INTERACTING = "interacting"
    SLEEPING = "sleeping"
    EMERGENCY_STOP = "emergency_stop"


class Emotion(Enum):
    HAPPY = "happy"
    CURIOUS = "curious"
    STARTLED = "startled"
    BORED = "bored"
    LONELY = "lonely"

#########################################################################
# Initialize pygame mixer for playing mp3 sounds
#########################################################################
pygame.mixer.init()
    
def load_sound(filename):
    return pygame.mixer.Sound(SOUNDS_PATH + filename)
    

########################################################################## Sounds dictionary for emotions 
##########################################################################
SOUNDS = {
    Emotion.HAPPY: load_sound("single_bark_1.mp3"),
    Emotion.CURIOUS: load_sound("woohoo.mp3"),
    Emotion.STARTLED: load_sound("growl_1.mp3"),
    Emotion.BORED: load_sound("snoring.mp3"),
    Emotion.LONELY: load_sound("howling.mp3"),
}

def play_sound(sound):
    if sound:
        sound.play()

class Ros2AutonomousPiDog(Node):
    def __init__(self):
        super().__init__('ros2_autonomous_pidog')
        
        # Parameters to be set false during debugging  
        self.declare_parameter('enable_wandering', True)
        self.declare_parameter('enable_obstacle_avoidance', True)
        
        self.enable_wandering = self.get_parameter('enable_wandering').value
        self.enable_obstacle_avoidance = self.get_parameter('enable_obstacle_avoidance').value

        # ============================================================
        # INITIALIZE PIDOG HARDWARE
        # ============================================================
        self.get_logger().info("=" * 60)
        self.get_logger().info("Initializing PiDog hardware...")
        
        self.pidog_manager = get_pidog_manager()
        
        if self.pidog_manager.initialize(disable_sensors=False):
            self.dog = self.pidog_manager.get_pidog()
            self.get_logger().info("✓ PiDog hardware available")
            
            try:
                self.get_logger().info("Testing hardware...")
                self.dog.do_action('sit', speed=50)
                self.dog.wait_all_done()
                time.sleep(0.5)
                self.dog.do_action('stand', speed=50)
                self.dog.wait_all_done()
                self.get_logger().info("✓ Hardware test passed")
            except Exception as e:
                self.get_logger().error(f"Hardware test failed: {e}")
                return
        else:
            self.dog = None
            self.get_logger().error("✗ PiDog hardware NOT available!")
            return
        
        self.imu_offset = self.calibrate_imu() # Calibrate IMU at startup for better accuracy at standing position

        self.sound_disabled = True  # ADD THIS STATE TRACKING
        self.sound_direction_client = self.create_client(SetBool, '/pidog/enable_sound_direction')
        # Wait for service (don't block startup)
        while not self.sound_direction_client.wait_for_service(timeout_sec=0.1):
            if rclpy.ok():
                self.get_logger().debug('Waiting for sound direction service...')
            else:
                break

        # Distance sensor filtering
        self.distance_readings = deque(maxlen=3)  # Store last 3 readings
        self.last_distance_read_time = 0
        self.distance_read_interval = 0.1  # Read every 100ms max
        self.distance_buffer = deque(maxlen=5)  # Buffer for incoming distance messages to filter spikes

        # Add after other buffers initialization:
        self.filtered_distance_history = deque(maxlen=5)  # Track filtered distance over time
        self.current_distance_filtered = 999.0
        self.current_distance_stable = 999.0

        # ============================================================
        # STATE MANAGEMENT
        # ============================================================
        self.state = RobotState.WANDERING
        self.emotion = Emotion.HAPPY
        self.last_emotion_time = time.time()
        self.emotion_interval = 30
        
        # Sensor data cache
        self.current_distance = 999.0
        self.current_imu = None
        self.last_touch = False
        self.last_touch_sensor = -1
        self.sound_direction_angle = -1.0
        self.sound_direction_text = "unknown"
        self.face_count = 0
        
        # Behavior tracking
        self.turn_history = deque(maxlen=5)
        
        # Control flags
        self.emergency_stop = False
        self.voice_command_waiting = False
        self.command_lock = threading.Lock()
        self.command_active = False
        
        # Parameters
        self.enable_sound_turning = True
        self.enable_face_interaction = True
        self.personality_actions = True
        
        self.is_picked_up = False
        self.pickup_debounce_time = 0
        self.pickup_cooldown = 3.0  # Prevent multiple triggers
        self.pickup_detection_enabled = True
        self.is_moving = False

        # ============================================================
        # ROS 2 PUBLISHERS & SUBSCRIBERS
        # ============================================================
        qos_best = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        
        # Publishers
        self.distance_pub = self.create_publisher(Float32, 'distance', qos_best)
        self.imu_pub = self.create_publisher(String, 'imu', qos_best)
        self.touch_pub = self.create_publisher(String, 'touch', qos_best)
        
        self.status_pub = self.create_publisher(String, 'status', qos_best)
        self.speak_pub = self.create_publisher(String, 'speak_text', qos_rel)
        
        self.sound_disabled = False
        self.dog.rgb_strip.set_mode(style="boom", color="#a10a0a", bps=2.5, brightness=0.5)

        #self._speak_timer = None
        self.speak(GREETING_EN+f"I am running on Ubuntu 22.04 with ROS 2 Humble")
        #time.sleep(8)  # Wait for first speech to complete
        #self.speak(f"I am running on Ubuntu 22.04 with ROS 2 Humble")

        # Subscribers (for external commands)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_rel)
        self.command_sub = self.create_subscription(String, 'command', self.command_callback, qos_rel)
        
        # Subscribe to FILTERED distance from distance processing node
        self.distance_filtered_sub = self.create_subscription(
            Float32, 
            'distance/filtered',  # Filtered distance with outlier rejection
            self.distance_filtered_callback, 
            qos_best
        )

        # Subscribe to STABLE distance for critical decisions
        self.distance_stable_sub = self.create_subscription(
            Float32, 
            'distance/stable',  # Even more stable (temporal consistency)
            self.distance_stable_callback, 
            qos_best
        )

        self.sound_pub = self.create_publisher(String, 'sound_direction', qos_best)

        # Battery monitoring subscriber
        self.battery_status_sub = self.create_subscription(
            String,
            'battery_status',
            self.battery_status_callback,
            qos_best
        )

        # Battery state variables
        self.current_battery_voltage = 0.0
        self.current_battery_percentage = 0.0
        self.battery_status = "unknown"

        # Direct voice command subscriber (bypasses voice_bridge for faster response)
        self.voice_sub = self.create_subscription(String, 'voice_command', self.voice_callback, qos_rel)
        
        # Sensor subscribers (from other nodes - for redundancy)
        self.distance_raw_pub = self.create_publisher(Float32, 'distance/raw', qos_best)
        self.imu_sub = self.create_subscription(String, 'imu', self.imu_callback, qos_best)
        self.touch_sub = self.create_subscription(String, 'touch', self.touch_callback, qos_best)
        self.sound_dir_sub = self.create_subscription(String, 'sound_direction', self.sound_direction_callback, qos_best)
        self.face_sub = self.create_subscription(String, 'face_detection', self.face_callback, qos_best)

        self.obstacle_debounce_time = 0
        self.obstacle_debounce_duration = 1.0  # 1 second debounce
        self.last_sound_reaction_time = 0
        self.sound_reaction_cooldown = 2.0  # Don't react to sound more than once per 2 seconds


        # ============================================================
        # START THREADS
        # ============================================================
        self.sensor_thread = threading.Thread(target=self.sensor_reading_loop, daemon=True)
        self.sensor_thread.start()
        
        self.autonomous_thread = threading.Thread(target=self.autonomous_behavior_loop, daemon=True)
        self.autonomous_thread.start()
        
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("ROS 2 Autonomous PiDog Node Ready!")
        self.get_logger().info("Features: Autonomous Wandering | Obstacle Avoidance | Voice Commands")
        self.get_logger().info("Say: sit, stand, walk, stop, turn left, turn right, resume, hand shake, high five, scratch")
        self.get_logger().info("=" * 60)
    
    def calibrate_imu(self):
        """Calibrate IMU gyroscope only - collect bias while stationary"""
        self.get_logger().info("Calibrating IMU gyroscope - keep dog still...")
        samples_gx, samples_gy, samples_gz = [], [], []
    
        for _ in range(100):
            try:
                # Use the correct method to get gyro data
                gx, gy, gz = self.dog.gyroData  # This returns gyroscope data
                samples_gx.append(gx)
                samples_gy.append(gy)
                samples_gz.append(gz)
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().debug(f"Calibration sample error: {e}")
    
        # Check if we have samples
        if len(samples_gx) == 0:
            self.get_logger().warning("No gyro samples collected, using zero offsets")
            return (0, 0, 0, 0, 0, 0)
    
        # Calculate gyro offsets only
        offset_gx = sum(samples_gx) / len(samples_gx)
        offset_gy = sum(samples_gy) / len(samples_gy)
        offset_gz = sum(samples_gz) / len(samples_gz)
    
        self.get_logger().info(f"IMU gyro calibration complete:")
        self.get_logger().info(f"  Gyro offsets: gx={offset_gx:.1f}, gy={offset_gy:.1f}, gz={offset_gz:.1f}")
    
        # Return zeros for accelerometer offsets (not used)
        return (0, 0, 0, offset_gx, offset_gy, offset_gz)

    # ============================================================
    # SENSOR READING LOOP (Direct hardware access)
    # ============================================================
    def sensor_reading_loop(self):
        """Read sensors directly from hardware and publish data"""
        self.get_logger().info("Sensor reading thread started")
        time.sleep(1)
        
        while rclpy.ok() and self.dog:
            try:
                current_time = time.time()
            
                # Read distance sensor - with rate limiting
                if hasattr(self.dog, 'ultrasonic') and hasattr(self.dog.ultrasonic, 'read'):
                    # Only read every 100ms to avoid noise
                    if current_time - self.last_distance_read_time >= self.distance_read_interval:
                        distance = self.dog.ultrasonic.read()
                        self.last_distance_read_time = current_time
                    
                        if distance and 2 <= distance <= 400:
                            # Add to rolling buffer for filtering
                            self.distance_readings.append(float(distance))
                        
                            # Moving average filtering - use MINIMUM for obstacle detection, MEDIAN for display:
                            if len(self.distance_readings) >= 3:
                                # For obstacle detection, use MINIMUM (most conservative)
                                # This triggers EARLIER when ANY reading is low
                                min_distance = min(self.distance_readings)
    
                                # Also keep median for general display (optional)
                                sorted_readings = sorted(self.distance_readings)
                                median_distance = sorted_readings[len(sorted_readings)//2]
                                # Use MIN for obstacle detection, MEDIAN for display
                                self.current_distance_for_obstacle = min_distance  # For obstacle checking
                                self.current_distance = median_distance  # For display/logging
    
                                if abs(median_distance - min_distance) > 10:
                                    self.get_logger().debug(f"Distance: min={min_distance:.1f}, median={median_distance:.1f}")
                            else:
                                self.current_distance = float(distance)
                                self.current_distance_for_obstacle = float(distance)
                        
                            self.distance_raw_pub.publish(Float32(data=float(distance)))
                        
                            # Log every few readings
                            if random.randint(1, 20) == 1:
                                self.get_logger().info(f"📏 Distance: {self.current_distance:.1f} cm")
                
                # Read IMU data - Publish to /imu topic for other nodes
                try:
                    # Get raw values and convert to G-force (1G = 16384)
                    ax_raw, ay_raw, az_raw = self.dog.accData
                    gx_raw, gy_raw, gz_raw = self.dog.gyroData
                
                    # Convert to G-force
                    ax = ax_raw / 16384.0
                    ay = ay_raw / 16384.0
                    az = az_raw / 16384.0
                
                    # Remap axes for dog's orientation:
                    # - Vertical (gravity) = -ax (points up, 1.0g when standing)
                    # - Forward/Backward = az (tilt detection)
                    # - Left/Right = ay (roll detection)
                
                    vertical_g = -ax  # Upward direction (gravity when standing)
                    forward_g = az    # Forward/backward tilt
                    right_g = ay      # Left/right tilt
                
                    # Calculate pitch (forward/back tilt) using forward and vertical
                    # When dog tilts forward, forward_g increases, vertical_g decreases
                    pitch = math.atan2(forward_g, vertical_g) * 180.0 / math.pi
                
                    # Calculate roll (left/right tilt) using right and vertical
                    roll = math.atan2(right_g, vertical_g) * 180.0 / math.pi
                
                    # Yaw would need magnetometer
                    yaw = 0.0
                
                    # Remap gyroscopes similarly (assuming same orientation)
                    # Gyro axes: X up/down, Y right/left, Z forward/back
                    gyro_vertical = -gx_raw  # Yaw rate (rotate around vertical)
                    gyro_forward = gz_raw    # Pitch rate (forward/back)
                    gyro_right = gy_raw      # Roll rate (left/right)
                
                    # Log occasionally
                    if random.randint(1, 20) == 1:
                        self.get_logger().debug(f"📊 IMU: fwd={forward_g:.3f}g, right={right_g:.3f}g, up={vertical_g:.3f}g")
                        self.get_logger().debug(f"   Angles: roll={roll:.1f}°, pitch={pitch:.1f}°")
                        self.get_logger().debug(f"   Magnitude: {math.sqrt(forward_g*forward_g + right_g*right_g + vertical_g*vertical_g):.3f}g")
                
                    # Create IMU message with RAW values included
                    imu_msg = String()
                    # Format: roll,pitch,yaw,accel_g,accel_g,accel_g,accel_raw,accel_raw,accel_raw,gyro,gyro,gyro
                    imu_msg.data = f"{roll:.2f},{pitch:.2f},{yaw:.2f},{forward_g:.3f},{right_g:.3f},{vertical_g:.3f},{ax_raw:.1f},{ay_raw:.1f},{az_raw:.1f},{gyro_forward:.1f},{gyro_right:.1f},{gyro_vertical:.1f}"
                    self.imu_pub.publish(imu_msg)
                
                    self.current_imu = {
                        'roll': roll,
                        'pitch': pitch,
                        'yaw': yaw,
                        'accel_x': forward_g,      # G-force for other uses
                        'accel_y': right_g,
                        'accel_z': vertical_g,
                        'accel_x_raw': ax_raw,     # RAW ADC values for pickup detection
                        'accel_y_raw': ay_raw,
                        'accel_z_raw': az_raw,
                        'gyro_x': gyro_forward,
                        'gyro_y': gyro_right,
                        'gyro_z': gyro_vertical
                    }
                
                except Exception as e:
                    self.get_logger().debug(f"IMU read error: {e}")
            
                # Read touch sensors - Corrected based on official documentation
                if hasattr(self.dog, 'dual_touch'):
                    try:
                        touch_result = self.dog.dual_touch.read()
        
                        # Check if a touch is detected (not "N")
                        if touch_result != 'N':
                            self.get_logger().info(f"👆 Touch detected: {touch_result}")
            
                            # Create a descriptive message for the touch topic
                            touch_msg = f"touched:{touch_result}:1.0"
                            self.touch_pub.publish(String(data=touch_msg))
            
                            # Call the touch event handler with the result string
                            self.handle_touch_event(touch_result)
            
                            # Debounce: wait a bit to avoid multiple rapid events
                            time.sleep(0.2)
            
                    except Exception as e:
                        self.get_logger().info(f"Touch read error: {e}")
                
                # Read sound direction
                if self.dog.ears.isdetected():
                    angle = self.dog.ears.read()
                    #if angle is not None and 0 <= angle <= 360:
                    self.sound_direction_angle = angle
                    self.sound_direction_text = self.angle_to_direction(angle)
                    self.sound_pub.publish(String(data=f"{angle:.1f}:{self.sound_direction_text}:1"))
                    self.get_logger().debug(f"🔊 Sound detected at {angle:.1f}° ({self.sound_direction_text})")
                    #time.sleep(0.1)
                # Small sleep to prevent CPU hogging
                time.sleep(0.05)  # 20ms - faster than before but with rate limiting above
            except Exception as e:
                self.get_logger().debug(f"Sensor read error: {e}")
                time.sleep(0.05)

    def angle_to_direction(self, angle):
        """Convert angle to direction string"""
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
    
    # ============================================================
    # SENSOR CALLBACKS (from other nodes)
    # ============================================================
    def distance_filtered_callback(self, msg: Float32):
        """Use filtered distance for obstacle detection - rejects outliers"""
        self.current_distance_filtered = msg.data
        self.filtered_distance_history.append(msg.data)  # Store for temporal checks
        self.get_logger().debug(f"Filtered distance: {msg.data:.2f} cm")

    def distance_stable_callback(self, msg: Float32):
        """Use stable distance for critical decisions - highest confidence"""
        self.current_distance_stable = msg.data

    def imu_callback(self, msg):
        try:
            parts = msg.data.split(',')
            if len(parts) >= 12:  # Extended format with raw values
                self.current_imu = {
                    'roll': float(parts[0]),
                    'pitch': float(parts[1]),
                    'yaw': float(parts[2]),
                    'accel_x': float(parts[3]),
                    'accel_y': float(parts[4]),
                    'accel_z': float(parts[5]),
                    'accel_x_raw': float(parts[6]),   # RAW ADC
                    'accel_y_raw': float(parts[7]),
                    'accel_z_raw': float(parts[8]),
                    'gyro_x': float(parts[9]),
                    'gyro_y': float(parts[10]),
                    'gyro_z': float(parts[11])
                }
                self.handle_pickup_detection()
            elif len(parts) >= 9:
                # Old format without raw
                self.current_imu = {
                    'roll': float(parts[0]),
                    'pitch': float(parts[1]),
                    'yaw': float(parts[2]),
                    'accel_x': float(parts[3]),
                    'accel_y': float(parts[4]),
                    'accel_z': float(parts[5]),
                    'gyro_x': float(parts[6]),
                    'gyro_y': float(parts[7]),
                    'gyro_z': float(parts[8])
                }
        except Exception as e:
            self.get_logger().debug(f"IMU callback error: {e}")
    
    
    def touch_callback(self, msg):
        """Handle touch messages from other nodes or from our own publisher"""
        try:
            parts = msg.data.split(':')
            if len(parts) >= 2:
                # New format: "touched:LS:1.0" or "touched:L:1.0" etc.
                if parts[0] == 'touched':
                    touch_value = parts[1]  # This is now a string like "L", "R", "LS", "RS"
                    self.get_logger().info(f"Touch callback received: {touch_value}")
                    self.handle_touch_event(touch_value)  # Pass the string directly
        except Exception as e:
            self.get_logger().debug(f"Touch callback error: {e}")
    
    def sound_direction_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_sound_reaction_time < self.sound_reaction_cooldown:
            return  # Ignore during cooldown
    
        self.last_sound_reaction_time = current_time
        try:
            parts = msg.data.split(':')
            if len(parts) >= 2:
                self.sound_direction_angle = float(parts[0])
                self.sound_direction_text = parts[1] if len(parts) > 1 else "unknown"
        except:
            pass
    
    def face_callback(self, msg):
        try:
            parts = msg.data.split(':')
            if len(parts) >= 1:
                self.face_count = int(parts[0])
                if self.face_count > 0 and self.state == RobotState.WANDERING and self.enable_face_interaction:
                    self.handle_face_detection()
        except:
            pass
    
    # ============================================================
    # MOVEMENT EXECUTION (with immediate stop)
    # ============================================================
    def execute_movement(self, command, step_count=5, speed=70):
        """Execute movement with immediate stop of current action"""
        if self.dog is None:
            return False
        
        self.is_moving = True  # Set moving flag

        with self.command_lock:
            self.command_active = True
        
        try:
            # Stop current movement immediately
            self.dog.body_stop()
            self.dog.wait_all_done()
            time.sleep(0.05)
            
            self.get_logger().info(f"➡️ {command}")
            
            if command == 'stop':
                # Already stopped, just return
                self.get_logger().info("Stop command executed")
                return True
            elif command in ['sit', 'stand']:
                self.dog.do_action(command, speed=speed)
                self.dog.wait_all_done()
            elif command in ['hand_shake']:
                self.dog.do_action('sit', speed=speed)
                self.dog.wait_all_done()
                hand_shake(self.dog)
                self.dog.wait_all_done()
            elif command in ['high_five']:
                self.dog.do_action('sit', speed=speed)
                self.dog.wait_all_done()
                high_five(self.dog)
                self.dog.wait_all_done()
            elif command in ['push_up']:
                self.dog.do_action(command, step_count=5, speed=speed)
                self.dog.wait_all_done()
                self.dog.do_action('sit', speed=speed)
                self.dog.wait_all_done()
            elif command in ['stretch']:
                self.dog.do_action(command, speed=speed)
                self.dog.wait_all_done()
                self.dog.do_action('stand', speed=speed)
                self.dog.wait_all_done()
            elif command in ['scratch']:
                self.dog.do_action('sit', speed=speed)
                self.dog.wait_all_done()        
                #scratch(self.dog)
                #head_angs = [ [0, 0, 0], [0, 0, 0] ]
                #self.dog.head_move_raw(head_angs, immediately=False, speed=80)
                time.sleep(2)
            else:
                steps = max(1, step_count) if step_count > 0 else 1
                self.dog.do_action(command, step_count=steps, speed=speed)
                self.dog.wait_all_done()
            return True
        except Exception as e:
            self.get_logger().error(f"Movement error: {e}")
            return False
        finally:
            time.sleep(0.2)
            with self.command_lock:
                self.command_active = False
            self.is_moving = False  # Clear moving flag
    
    # ============================================================
    # MOVEMENT COMMANDS (for internal use)
    # ============================================================
    def send_move_command(self, command, step_count=0, speed=70):
        """Send movement command"""
        self.execute_movement(command, step_count, speed)
    
    # ============================================================
    # VOICE COMMAND HANDLING (Direct - Immediate response)
    # ============================================================
    def voice_callback(self, msg):
        """Handle voice commands - IMMEDIATE execution"""
        # In voice_callback, add command aliases:
        self.command_map = {
            'sit': ['sit', 'sit down', 'set', 'settle'],
            'stand': ['stand', 'stand up', 'set up', 'sen up','sten', 'stem'],
            'walk': ['walk', 'forward', 'go'],
            'stop': ['stop', 'halt', 'freeze'],
        }
        text = msg.data.lower().strip()
        
        # Filter garbage
        if len(text) < 2:
            return
        
        false_positives = ['ah', 'uh', 'um', 'oh', 'by', 'do', 'go', 'to', 'be', 'me', 
                   'wow', 'the bow', 'oops', 'it\'s now', 'the out']
        if text in false_positives:
            return
        
        # INTERRUPT CURRENT ACTION - Voice commands have priority!
        with self.command_lock:
            if self.command_active:
                self.get_logger().info("🛑 Interrupting current action for voice command")
                # Force stop the dog immediately
                if self.dog:
                    self.dog.body_stop()
                    self.dog.wait_all_done()
                self.command_active = False  # Release the lock
    
        # Small delay to ensure stop completes
        time.sleep(0.1)
    
        # Now acquire lock for new command
        with self.command_lock:
            self.command_active = True
        
        self.get_logger().info(f"🎤 Voice command: '{text}'")
        self.voice_command_waiting = True
        
        # Process command
        try:
            if 'sit' in text:
                self.get_logger().info("📢 SIT")
                # Clear any pending commands first
                self.dog.body_stop()
                self.dog.wait_all_done()
                time.sleep(0.1)
                self.execute_movement('sit', step_count=0, speed=60)  # sit doesn't need step_count
                self.speak("Sitting down")
                self.state = RobotState.INTERACTING
                threading.Timer(5.0, self.return_to_wandering).start()
            
            elif 'stand' in text:
                self.get_logger().info("📢 STAND")
                self.execute_movement('stand')
                self.speak("Standing up")
                self.state = RobotState.INTERACTING
                threading.Timer(5.0, self.return_to_wandering).start()
            
            elif 'walk' in text or 'forward' in text:
                self.get_logger().info("📢 WALK")
                self.execute_movement('forward', step_count=12, speed=FORWARD_SPEED)
            
            elif 'back' in text:
                self.get_logger().info("📢 BACK")
                self.execute_movement('backward', step_count=4, speed=80)
            
            elif 'stop' in text and len(text) < 10:
                self.get_logger().info("📢 STOP")
                self.execute_movement('stop')
                self.speak("Stopping")
                self.state = RobotState.INTERACTING
                
            elif 'resume' in text and len(text) < 10:
                self.get_logger().info("📢 RESUME WANDERING")
                self.speak("Resuming wandering")
                self.state = RobotState.WANDERING
                threading.Timer(2.0, self.return_to_wandering).start()
            
            elif 'left' in text and (len(text) < 10 or 'turn left' in text):
                self.get_logger().info("📢 TURN LEFT")
                self.execute_movement('turn_left', step_count=TURN_STEPS, speed=TURN_SPEED)
            
            elif 'right' in text and (len(text) < 10 or 'turn right' in text):
                self.get_logger().info("📢 TURN RIGHT")
                self.execute_movement('turn_right', step_count=TURN_STEPS, speed=TURN_SPEED)
            elif "stretch" in text:
                self.get_logger().info("📢 STRETCH")
                self.execute_movement('stretch', step_count=0, speed=60)
            elif "hand" in text or "shake" in text or "handshake" in text:
                self.get_logger().info("📢 Hand Shake")
                # Clear any pending commands first
                self.dog.body_stop()
                self.dog.wait_all_done()
                time.sleep(0.1)
                self.execute_movement('hand_shake', step_count=0, speed=80)  
                self.speak("Hand Shake")
                self.state = RobotState.INTERACTING
                #threading.Timer(5.0, self.return_to_wandering).start()
            elif "scratch" in text:
                self.get_logger().info("📢 Scratch")
                # Clear any pending commands first
                self.dog.body_stop()
                self.dog.wait_all_done()
                time.sleep(0.1)
                self.execute_movement('scratch')  
                self.speak("Scratch")
                self.state = RobotState.INTERACTING
                #threading.Timer(5.0, self.return_to_wandering).start()
            elif "high" in text or "five" in text or "high five" in text:
                self.get_logger().info("📢 High Five")
                # Clear any pending commands first
                self.dog.body_stop()
                self.dog.wait_all_done()
                time.sleep(0.1)
                self.execute_movement('high_five',step_count=0, speed=80)  
                self.speak("High Five")
                self.state = RobotState.INTERACTING
                #threading.Timer(5.0, self.return_to_wandering).start()
            elif "push" in text or "up" in text or "push up" in text:
                self.get_logger().info("📢 Push Up")
                # Clear any pending commands first
                self.dog.body_stop()
                self.dog.wait_all_done()
                time.sleep(0.1)
                self.execute_movement('push_up',step_count=0, speed=80)  
                self.speak("Push Up")
                self.state = RobotState.INTERACTING
            else:
                self.get_logger().info(f"Unknown command: '{text}'")
            
        finally:
            self.voice_command_waiting = False
            with self.command_lock:
                self.command_active = False
    
    
    # ============================================================
    # BATTERY MONITORING (Placeholder - battery monitor not running yet)
    # ====================================info========================
    def battery_status_callback(self, msg: String):
        """Receive battery status from battery monitor node"""
        try:
            # Format: "voltage:7.5:percentage:85:status:good"
            parts = msg.data.split(':')
            if len(parts) >= 6:
                self.current_battery_voltage = float(parts[1])
                self.current_battery_percentage = float(parts[3])
                self.battery_status = parts[5]
            
                if self.battery_status == "critical":
                    self.get_logger().error(f"🔴 CRITICAL: Battery {self.current_battery_voltage:.2f}V!")
                    self.speak("Battery is critically low! Please charge me!", use_emotion=True)
                if self.current_battery_percentage < 15:
                    self.state = RobotState.SLEEPING
                    self.speak("Battery low, going to sleep")
                    self.execute_movement('sit')
        except Exception as e:
            self.get_logger().debug(f"Battery callback error: {e}")

    def speak(self, text, use_emotion=False):
        """Send speech command to TTS node without causing echo feedback"""

        # Don't disable if already disabled
        if self.sound_disabled:
            self.get_logger().debug("Sound already disabled, skipping duplicate request")
            # Still publish speech but don't call service again
            cmd_str = f"{text}:{1 if use_emotion else 0}"
            msg = String()
            msg.data = cmd_str
            self.speak_pub.publish(msg)
            return
    
        # Step 1: Disable sound direction BEFORE speaking
        req = SetBool.Request()
        req.data = False
        self.sound_direction_client.call_async(req)
        self.sound_disabled = True  # ADD THIS STATE TRACKING
    
        # Small delay to ensure service processes
        time.sleep(0.05)
    
        # Step 2: Speak using original functionality
        cmd_str = f"{text}:{1 if use_emotion else 0}"
        msg = String()
        msg.data = cmd_str
        self.speak_pub.publish(msg)
    
        # Step 3: Calculate duration for speech
        duration = len(text) * 0.15 + 2.5
        self.get_logger().info(f"🔇 Sound disabled for {duration:.1f}s")
    
        # Step 4: Cancel any existing timer
        if hasattr(self, '_speak_timer') and self._speak_timer:
            self._speak_timer.cancel()
            self._speak_timer = None

        # Step 5: Use ROS timer (MORE RELIABLE than threading.Timer)
        self._speak_timer = self.create_timer(duration, self._re_enable_sound)

    def _re_enable_sound(self):
        """Re-enable sound direction after speech"""
        # Only re-enable if we're not about to speak again
        if hasattr(self, '_speak_timer'):
            self._speak_timer = None
        req = SetBool.Request()
        req.data = True
        self.sound_direction_client.call_async(req)
        self.get_logger().debug("🔊 Sound re-enabled after speech")

    # ============================================================
    # EXTERNAL COMMAND HANDLERS
    # ============================================================
    def cmd_vel_callback(self, msg):
        """Handle velocity commands (teleoperation)"""
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
    
    def command_callback(self, msg):
        """Handle external command strings"""
        self.get_logger().info(f"External command: {msg.data}")
        parts = msg.data.lower().split(':')
        command = parts[0]
        step_count = int(parts[1]) if len(parts) > 1 else 5
        speed = int(parts[2]) if len(parts) > 2 else 70
        self.execute_movement(command, step_count, speed)
    
    # ============================================================
    # AUTONOMOUS BEHAVIORS
    # ============================================================
    def publish_status(self):
        """Publish robot status"""
        with self.command_lock:
            active = self.command_active
        status_str = f"state:{self.state.value}:emotion:{self.emotion.value}:distance:{self.current_distance:.1f}:busy:{active}"
        self.status_pub.publish(String(data=status_str))
    
    
    def handle_touch_event(self, touch_result):
        """React to touch sensor events based on official API."""
        # Only react if the dog is in a state where it can be interrupted
        if self.state != RobotState.WANDERING:
            return
    
        # Change state so we don't get interrupted by other behaviors
        self.state = RobotState.INTERACTING
    
        # Customize reaction based on touch type
        if touch_result == 'L':
            self.speak("You touched my left side", use_emotion=True)
            self.execute_movement("shake_head", step_count=3, speed=80)
        elif touch_result == 'R':
            self.speak("You touched my right side", use_emotion=True)
            self.execute_movement("shake_head", step_count=3, speed=80)
        elif touch_result == 'LS':
            self.speak("You petted me from front to back", use_emotion=True)
        elif touch_result == 'RS':
            self.speak("You petted me from back to front", use_emotion=True)
        else:
            self.speak("That tickles! Hehe!", use_emotion=True)
    
        # Perform a happy action
        self.execute_movement("wag_tail", step_count=5, speed=80)
    
        # Return to wandering after a few seconds
        threading.Timer(8.0, self.return_to_wandering).start()

    def handle_face_detection(self):
        """React to detected faces"""
        if self.face_count > 0 and self.state == RobotState.WANDERING:
            self.state = RobotState.INTERACTING
            self.get_logger().info(f"Detected {self.face_count} face(s)")
            self.execute_movement("stop")
            self.speak(f"Hi there! I see {self.face_count} of you!", use_emotion=True)
            self.execute_movement("wag_tail", step_count=5, speed=90)
            threading.Timer(3.0, self.return_to_wandering).start()

    def handle_pickup_detection(self):
        """Detect when dog is picked up using RAW accelerometer values - only when stationary"""
        if not self.pickup_detection_enabled or self.current_imu is None:
            return
    
        # ===== CHECK IF DOG IS MOVING =====
        is_moving = self.command_active or self.state == RobotState.WANDERING
    
        # If moving and not already picked up, skip detection
        if is_moving and not self.is_picked_up:
            if hasattr(self, 'pickup_debounce_counter'):
                self.pickup_debounce_counter = 0
            return
        # ==================================
    
        current_time = time.time()
    
        # Check cooldown
        if current_time - self.pickup_debounce_time < self.pickup_cooldown:
            return
    
        # Get RAW accelerometer values
        ax_raw = self.current_imu.get('accel_x_raw', 0)
        ay_raw = self.current_imu.get('accel_y_raw', 0)
        az_raw = self.current_imu.get('accel_z_raw', 0)
    
        # Debounce counter
        if not hasattr(self, 'pickup_debounce_counter'):
            self.pickup_debounce_counter = 0
    
        # LOG RAW VALUES EVERY SECOND for debugging
        if int(current_time) % 1 == 0 and int(current_time * 10) % 10 == 0:
            self.get_logger().debug(f"🔍 RAW: ax={ax_raw:.0f}, ay={ay_raw:.0f}, az={az_raw:.0f}, moving={is_moving}, picked={self.is_picked_up}")
    
        # ADJUSTED THRESHOLDS - Use values from your test
        # When standing: ax ≈ -16000 to -17500
        # When picked up: ax > -14000 (less negative)
        PICKUP_THRESHOLD = -13000  # Changed from -13000
        GROUND_THRESHOLD = -15000   # Changed from -15000 (or use -16000)

        # Detection logic using RAW values
        if not self.is_picked_up:
            # Check if picked up (ax > PICKUP_THRESHOLD)
            if ax_raw > PICKUP_THRESHOLD:
                self.pickup_debounce_counter += 1
                self.get_logger().info(f"📊 Debounce: {self.pickup_debounce_counter}/3, ax_raw={ax_raw:.0f}")
                if self.pickup_debounce_counter >= 2:
                    self.get_logger().info(f"🚀 PICKUP DETECTED! ax_raw={ax_raw:.0f} (stationary)")
                    self.is_picked_up = True
                    self.pickup_debounce_counter = 0
                    self.pickup_debounce_time = current_time
                    self.execute_fly_action()
            else:
                self.pickup_debounce_counter = 0
    
        # Update the return to ground section in handle_pickup_detection:
        elif self.is_picked_up:
            # Check if returned to ground (ax < GROUND_THRESHOLD)
            if ax_raw < GROUND_THRESHOLD:
                self.pickup_debounce_counter += 1
                if self.pickup_debounce_counter >= 2:
                    self.get_logger().info(f"📍 RETURNED TO GROUND! ax_raw={ax_raw:.0f}")
                    self.is_picked_up = False
                    self.pickup_debounce_counter = 0
                    self.pickup_debounce_time = current_time
        
                    try:
                        self.dog.body_stop()
                        self.dog.wait_all_done()
                        self.execute_movement('stand', speed=60)
                        self.dog.wait_legs_done()
                        self.dog.rgb_strip.set_mode('breath', color='green', bps=1)
                        self.state = RobotState.WANDERING  # Reset to wandering
                        self.command_active = False  # Clear command flag
                        self.voice_command_waiting = False  # Clear voice flag
                        self.get_logger().info("✅ Dog returned to ground and standing - ready for commands")
                    except Exception as e:
                        self.get_logger().error(f"Error returning to stand: {e}")
            else:
                self.pickup_debounce_counter = 0

    def execute_fly_action(self):
        """Execute the fly action when dog is picked up - from 6_be_picked_up.py"""
        self.get_logger().info("🐕 Flying action triggered!")
    
        # Interrupt current actions
        with self.command_lock:
            if self.command_active:
                self.dog.body_stop()
                self.dog.wait_all_done()
                self.command_active = False
    
        try:
            # Fly action sequence from 6_be_picked_up.py
            # 1. RGB effect
            self.dog.rgb_strip.set_mode('boom', color='red', bps=3)
        
            # 2. Leg movement - specific pose for flying
            self.dog.legs.servo_move([45, -45, 90, -80, 90, 90, -90, -90], speed=60)
        
            # 3. Wag tail
            self.dog.do_action('wag_tail', step_count=10, speed=100)
        
            # 4. Speak
            self.speak("Woo hoo! I'm flying!", use_emotion=True)
        
            # Wait for legs to complete
            self.dog.wait_legs_done()
            time.sleep(1)
        
            # Store state to prevent wandering while in air
            self.original_state = self.state
            self.state = RobotState.INTERACTING
        
        except Exception as e:
            self.get_logger().error(f"Error in fly action: {e}")

    def smart_turn(self):
        """Intelligent turning based on turn history"""
        with self.command_lock:
            if self.command_active:
                return "left"
        
        if self.turn_history.count("left") >= 3:
            direction = "right"
        elif self.turn_history.count("right") >= 3:
            direction = "left"
        else:
            if self.sound_direction_angle > 0:
                direction = "left" if self.sound_direction_angle < 180 else "right"
            else:
                direction = "left" if random.random() > 0.5 else "right"
        
        self.get_logger().info(f"🔄 Smart turning {direction} (steps={TURN_STEPS}, speed={TURN_SPEED})")
        # Increase step_count to 12-15 for a proper 90-degree turn
        self.execute_movement(f"turn_{direction}", step_count=TURN_STEPS, speed=TURN_SPEED)
        self.dog.wait_all_done()
        self.turn_history.append(direction)
        return direction
    
    def obstacle_avoidance(self):
        """Handle obstacle avoidance behavior using FILTERED distance"""
    
        # Use filtered distance (from distance node) for decision making
        # Fall back to raw if filtered not available
        if hasattr(self, 'current_distance_filtered'):
            distance_to_use = self.current_distance_filtered
        else:
            distance_to_use = self.current_distance  # fallback
    
        # Debounce - don't react too frequently
        current_time = time.time()
        if current_time - self.obstacle_debounce_time < self.obstacle_debounce_duration:
            self.get_logger().debug("Obstacle avoidance debounced")
            return
    
        with self.command_lock:
            if self.command_active or self.emergency_stop:
                return
    
        # Use stable distance for critical obstacles (more conservative)
        if hasattr(self, 'current_distance_stable'):
            # If stable distance shows obstacle, react immediately
            if self.current_distance_stable < (OBSTACLE_DISTANCE_CM - 5):
                self.get_logger().info(f"🚨 CRITICAL obstacle! Stable distance: {self.current_distance_stable:.1f}cm")
                distance_to_use = self.current_distance_stable
    
        # Only proceed if distance is TRULY close (using filtered readings)
        if distance_to_use >= OBSTACLE_DISTANCE_CM:
            self.get_logger().debug(f"Distance OK: {distance_to_use:.1f}cm")
            return
    
        self.obstacle_debounce_time = current_time
    
        # Additional check: require multiple filtered readings below threshold
        if hasattr(self, 'filtered_distance_history'):
            recent_filtered = list(self.filtered_distance_history)[-3:]
            if len(recent_filtered) >= 3:
                # Require at least 2 of last 3 filtered readings to be below threshold
                below_threshold = sum(1 for d in recent_filtered if d < OBSTACLE_DISTANCE_CM)
                if below_threshold < 2:
                    self.get_logger().debug(f"Ignoring transient obstacle: filtered readings {recent_filtered}")
                    return
    
        self.state = RobotState.AVOIDING
        self.get_logger().info(f"🚧 Obstacle confirmed! Distance: {distance_to_use:.1f} cm")
    
        # Emergency stop
        if self.dog:
            self.dog.body_stop()
            self.dog.wait_all_done()
        time.sleep(0.1)

        if self.enable_obstacle_avoidance:
            # Back up quickly - increase steps for more distance
            self.execute_movement('backward', step_count=10, speed=BACKWARD_SPEED)
            time.sleep(0.2)
    
            # Turn (now faster with updated smart_turn)
            self.smart_turn()
            self.play_emotion(Emotion.STARTLED)
    
        self.state = RobotState.WANDERING


    def play_emotion(self, emotion):
        """Express an emotion through sound and movement"""
        emotion_sounds = {
            Emotion.HAPPY: "Happy bark!",
            Emotion.CURIOUS: "Hmm? What's that?",
            Emotion.STARTLED: "Woof! That scared me!",
            Emotion.BORED: "Sigh... I'm bored",
            Emotion.LONELY: "Awooo... anyone there?"
        }
        
        if emotion in emotion_sounds and not self.voice_command_waiting:
        #    self.speak(emotion_sounds[emotion], use_emotion=True)
            sound = SOUNDS.get(emotion)
            self.emotion = emotion
            
            self.get_logger().info(f"Emotion triggered: {emotion}")
            if sound:
                play_sound(sound)
            if emotion == Emotion.HAPPY:
                self.dog.head_move([(0, 0, 10)], immediately=True, speed=100)
                self.execute_movement("wag_tail", step_count=3, speed=90)
            elif emotion == Emotion.CURIOUS:
                self.dog.head_move([(-20, 0, 10)], immediately=True, speed=80)
                time.sleep(0.5)
                self.dog.head_move([(20, 0, 10)], immediately=True, speed=80)
                time.sleep(0.5)
                self.dog.head_move([(0, 0, 10)], immediately=True, speed=80)
                #self.execute_movement("wag_tail", step_count=0, speed=50) # head_tilt
            elif emotion == Emotion.STARTLED:
                self.dog.do_action("shake_head")
                #self.execute_movement("wag_tail", step_count=0, speed=80) # startle
            elif emotion == Emotion.BORED:
                # maybe a slow nod ?
                self.execute_movement("wag_tail", step_count=0, speed=80) # startle
            elif emotion == Emotion.LONELY:
                # maybe a slow wag tail or look around?
                self.execute_movement("wag_tail", step_count=0, speed=80) # startle
            
    
    def return_to_wandering(self):
        """Return to wandering state after interaction"""
        if not self.emergency_stop:
            self.state = RobotState.WANDERING
            self.get_logger().info("Returning to wandering mode")

    

    def random_personality_action(self):
        """Perform random action for personality"""
        with self.command_lock:
            if self.command_active:
                return
        
        #actions = [
        #    ("scratch", 3, 70),
        #    ("shake_head", 3, 80),
        #    ("wag_tail", 5, 90),
        #    ("look_around", 0, 50),
        #]
        
        #action = random.choice(actions)
        #self.get_logger().info(f"🎭 Random personality: {action[0]}")
        #self.execute_movement(action[0], action[1], action[2])
        self.show_some_personality(random.randrange(1,100))
        
    # ============================================================
    # AUTONOMOUS BEHAVIOR LOOP
    # ============================================================
    def autonomous_behavior_loop(self):
        """Main autonomous behavior loop"""
        self.get_logger().info("Autonomous behavior loop started")
        
        # Stand up initially
        self.execute_movement("stand")
        time.sleep(2)
        
        last_personality_time = time.time()
        personality_interval = 20
        
        while rclpy.ok():
            try:
                # Check for emergency stop
                if self.emergency_stop:
                    time.sleep(0.5)
                    continue
                
                # ===== ADD PICKUP CHECK =====
                # Don't wander if picked up
                if self.is_picked_up:
                    time.sleep(0.2)  # Short sleep while in air
                    continue
                # ===========================

                # Check if voice command is active
                with self.command_lock:
                    if self.command_active:
                        time.sleep(0.1)
                        continue
                
                # Different behaviors based on state
                if self.state == RobotState.WANDERING:
                    # Check for obstacles
                    #if self.current_distance and self.current_distance < OBSTACLE_DISTANCE_CM:
                    #    self.obstacle_avoidance()
                    #else:
                    # Use filtered distance (self.current_distance is already filtered)
                    # Also require at least 3 readings for confidence
                    if hasattr(self, 'current_distance_filtered'):
                        if self.current_distance_filtered < OBSTACLE_DISTANCE_CM:
                            self.obstacle_avoidance()
                        else:
                            if self.enable_wandering:
                                self.is_moving = True
                                # Move forward normally
                                self.execute_movement("forward", step_count=8, speed=FORWARD_SPEED)  # Increased step_count
                                self.is_moving = False
                    
                            # Check obstacle during movement (most critical part)
                            # Poll distance sensor while moving
                            check_interval = 0.15  # Check every 0.15 seconds
                            #checks_during_move = 4  # Total movement duration: 0.6 seconds

                            for check in range(3):  # Reduced checks
                                time.sleep(check_interval)
                                if len(self.distance_readings) >= 3 and self.current_distance_for_obstacle < OBSTACLE_DISTANCE_CM:
                                    self.get_logger().info("🚧 Obstacle detected while moving!")
                                    self.dog.body_stop()
                                    self.dog.wait_all_done()
                                    self.obstacle_avoidance()
                                    break
                    
                            # Random personality actions occasionally
                            now = time.time()
                            if now - last_personality_time > personality_interval and self.personality_actions:
                                if random.random() < 0.3:
                                    self.random_personality_action()
                                    last_personality_time = now
                        
                            # Periodic emotions
                            now = time.time()
                            if now - self.last_emotion_time > self.emotion_interval:
                                emotions = list(Emotion)
                                random_emotion = random.choice(emotions)
                                self.play_emotion(random_emotion)
                                self.last_emotion_time = now
                    
                        #time.sleep(0.3)
                    
                elif self.state == RobotState.SLEEPING:
                    time.sleep(1.0)
                else:
                    time.sleep(0.1)
                    
            except Exception as e:
                self.get_logger().error(f"Error in autonomous behavior: {e}")
                time.sleep(1.0)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down...")
        if self.dog:
            try:
                self.execute_movement('sit')
                self.pidog_manager.shutdown()
            except:
                pass
    
    def show_some_personality(self, RanAction):
        if   RanAction==1 : pidog.preset_actions.pant(self.dog)
        #if   RanAction==1 : pidog.preset_actions.scratch(self.dog)
        #elif RanAction==2 : preset_actions.hand_shake(self.dog)
        elif RanAction==3 : pidog.preset_actions.high_five(self.dog)
        elif RanAction==4 : pidog.preset_actions.pant(self.dog)
        elif RanAction==5 : pidog.preset_actions.body_twisting(self.dog)
        elif RanAction==6 : pidog.preset_actions.bark_action(self.dog)
        elif RanAction==7 : pidog.preset_actions.shake_head(self.dog)
        elif RanAction==8 : pidog.preset_actions.shake_head_smooth(self.dog)
        #elif RanAction==1 : bark(self.dog)
        #elif RanAction==1 : push_up(self.dog)
        elif RanAction==9 : pidog.preset_actions.howling(self.dog)
        elif RanAction==10: pidog.preset_actions.attack_posture(self.dog)
        elif RanAction==11: pidog.preset_actions.lick_hand(self.dog)
        elif RanAction==12: pidog.preset_actions.waiting(self.dog,0)#no def pitch for some reason
        elif RanAction==13: pidog.preset_actions.feet_shake(self.dog)
        elif RanAction==14: pidog.preset_actions.sit_2_stand(self.dog)
        elif RanAction==15: pidog.preset_actions.relax_neck(self.dog)
        elif RanAction==16: pidog.preset_actions.nod(self.dog)
        elif RanAction==17: pidog.preset_actions.think(self.dog)
        elif RanAction==18: pidog.preset_actions.recall(self.dog)
        elif RanAction==19: pidog.preset_actions.head_down_left(self.dog)
        elif RanAction==20: pidog.preset_actions.head_down_right(self.dog)
        elif RanAction==21: pidog.preset_actions.fluster(self.dog)
        elif RanAction==22: pidog.preset_actions.alert(self.dog)
        elif RanAction==23: pidog.preset_actions.surprise(self.dog)
        elif RanAction==24: pidog.preset_actions.stretch(self.dog)
        #Several likelihoods for turning, as it helps reduce the long straight walk
        #until it sees a wall
        elif RanAction==27: self.dog.do_action("turn_left", speed=98)
        elif RanAction==28: self.dog.do_action("turn_left", speed=98)
        elif RanAction==29: self.dog.do_action("turn_right", speed=98)
        elif RanAction==30: self.dog.do_action("turn_right", speed=98)
        print("Action is",RanAction)
    
        #put head back after any actions such that ultrasonic is pointing straight ahead
        #head_angs = [ [0, 0, 0], [0, 0, 0] ]
        #self.dog.head_move_raw(head_angs, immediately=False, speed=80)
        self.dog.wait_all_done()

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
        node.get_logger().info("Shutting down autonomous Pi Dog node")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
