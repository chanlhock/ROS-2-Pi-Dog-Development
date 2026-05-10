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
#
# ros2_autonomous_pidog.py is licensed under the GNU General Public 
# License v3.0 Permissions of this strong copyleft license are 
# conditioned on making available complete source code of licensed 
# works and modifications, which include larger works using a licensed 
# work, under the same license. Copyright and license notices must be 
# preserved. Contributors provide an express grant of patent rights.
##########################################################################
"""
#from sys import platform

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

import threading
import time
import random
from collections import deque
from enum import Enum

# Import PiDog Manager
from .pidog_manager import get_pidog_manager

# Constants
# Pi Dog's name
NAME = "Woofer" # Name of the dog during my childhood
GREETING_EN = f"Hi, I am {NAME}. Your obedient ROS2 Pi Dog"
OBSTACLE_DISTANCE_CM = 30
FORWARD_SPEED = 80
TURN_SPEED = 70
BACKWARD_TIME = 1.0
TURN_TIME = 0.6

# Global Accessible PiDog Instance  
pidog_instance = None  # This will be set by the main node and can be accessed by other nodes if needed (use with caution)

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


class Ros2AutonomousPiDog(Node):
    def __init__(self):
        super().__init__('ros2_autonomous_pidog')
        
        # ============================================================
        # INITIALIZE PIDOG HARDWARE
        # ============================================================
        self.get_logger().info("=" * 60)
        self.get_logger().info("Initializing PiDog hardware...")
        
        self.pidog_manager = get_pidog_manager()
        
        if self.pidog_manager.initialize(disable_sensors=True):
            self.dog = self.pidog_manager.get_pidog()
            global pidog_instance
            pidog_instance = self.dog  # Set global instance for direct access in other nodes if needed
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
        
        # ============================================================
        # ROS 2 PUBLISHERS & SUBSCRIBERS
        # ============================================================
        qos_best = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        
        # Publishers
        self.distance_pub = self.create_publisher(Float32, 'distance', qos_best)
        self.imu_pub = self.create_publisher(String, 'imu', qos_best)
        self.touch_pub = self.create_publisher(String, 'touch', qos_best)
        self.sound_pub = self.create_publisher(String, 'sound_direction', qos_best)
        self.status_pub = self.create_publisher(String, 'status', qos_best)
        self.speak_pub = self.create_publisher(String, 'speak_text', qos_rel)
        
        # Subscribers (for external commands)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_rel)
        self.command_sub = self.create_subscription(String, 'command', self.command_callback, qos_rel)
        
        # Direct voice command subscriber (bypasses voice_bridge for faster response)
        self.voice_sub = self.create_subscription(String, 'voice_command', self.voice_callback, qos_best)
        
        # Sensor subscribers (from other nodes - for redundancy)
        self.distance_sub = self.create_subscription(Float32, 'distance', self.distance_callback, qos_best)
        self.imu_sub = self.create_subscription(String, 'imu', self.imu_callback, qos_best)
        self.touch_sub = self.create_subscription(String, 'touch', self.touch_callback, qos_best)
        self.sound_dir_sub = self.create_subscription(String, 'sound_direction', self.sound_direction_callback, qos_best)
        self.face_sub = self.create_subscription(String, 'face_detection', self.face_callback, qos_best)
        
        self.dog.rgb_strip.set_mode(style="boom", color="#a10a0a", bps=2.5, brightness=0.5)
        self.speak(GREETING_EN)
        time.sleep(1)
        self.speak(f"I am running on Ubuntu 22.04 with ROS 2 Humble")
        time.sleep(1)

        # ============================================================
        # START THREADS
        # ============================================================
        self.sensor_thread = threading.Thread(target=self.sensor_reading_loop, daemon=True)
        self.sensor_thread.start()
        
        self.autonomous_thread = threading.Thread(target=self.autonomous_behavior_loop, daemon=True)
        #self.autonomous_thread.start()
        
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("ROS 2 Autonomous PiDog Node Ready!")
        self.get_logger().info("Features: Autonomous Wandering | Obstacle Avoidance | Voice Commands")
        self.get_logger().info("Say: sit, stand, walk, stop, turn left, turn right")
        self.get_logger().info("=" * 60)
    
    # ============================================================
    # SENSOR READING LOOP (Direct hardware access)
    # ============================================================
    def sensor_reading_loop(self):
        """Read sensors directly from hardware and publish data"""
        self.get_logger().info("Sensor reading thread started")
        time.sleep(1)
        
        while rclpy.ok() and self.dog:
            try:
                # Read distance sensor
                if hasattr(self.dog, 'ultrasonic') and hasattr(self.dog.ultrasonic, 'read_distance'):
                    distance = self.dog.ultrasonic.read_distance()
                    if distance and 2 <= distance <= 400:
                        self.current_distance = float(distance)
                        self.distance_pub.publish(Float32(data=self.current_distance))
                
                # Read IMU data
                if hasattr(self.dog, 'get_imu_data'):
                    imu_data = self.dog.get_imu_data()
                    if imu_data:
                        imu_msg = String()
                        imu_msg.data = f"{imu_data.get('roll',0):.2f},{imu_data.get('pitch',0):.2f},{imu_data.get('yaw',0):.2f}"
                        self.imu_pub.publish(imu_msg)
                        self.current_imu = imu_data
                
                # Read touch sensors
                if hasattr(self.dog, 'dual_touch'):
                    for i in range(4):
                        if hasattr(self.dog.dual_touch, 'read'):
                            touched = self.dog.dual_touch.read(i)
                            if touched:
                                self.touch_pub.publish(String(data=f"touched:{i}:1.0"))
                                self.get_logger().info(f"Touch detected on sensor {i}")
                                self.handle_touch_event(i)
                
                # Read sound direction
                if hasattr(self.dog, 'ears'):
                    if hasattr(self.dog.ears, 'isdetected') and self.dog.ears.isdetected():
                        if hasattr(self.dog.ears, 'read'):
                            angle = self.dog.ears.read()
                            if angle is not None and 0 <= angle <= 360:
                                self.sound_direction_angle = angle
                                self.sound_direction_text = self.angle_to_direction(angle)
                                self.sound_pub.publish(String(data=f"{angle:.1f}:{self.sound_direction_text}:1"))
                
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().debug(f"Sensor read error: {e}")
                time.sleep(0.5)

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
    def distance_callback(self, msg):
        self.current_distance = msg.data
    
    def imu_callback(self, msg):
        try:
            parts = msg.data.split(',')
            if len(parts) >= 3:
                self.current_imu = {
                    'roll': float(parts[0]),
                    'pitch': float(parts[1]),
                    'yaw': float(parts[2]),
                }
        except:
            pass
    
    def touch_callback(self, msg):
        try:
            parts = msg.data.split(':')
            if len(parts) >= 2:
                touched = parts[0] == 'touched'
                sensor_id = int(parts[1])
                if touched:
                    self.handle_touch_event(sensor_id)
        except:
            pass
    
    def sound_direction_callback(self, msg):
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
        
        with self.command_lock:
            self.command_active = True
        
        try:
            # Stop current movement immediately
            self.dog.body_stop()
            self.dog.wait_all_done()
            time.sleep(0.05)
            
            self.get_logger().info(f"➡️ {command}")
            
            if command == 'stop':
                pass
            elif command in ['sit', 'stand']:
                self.dog.do_action(command, speed=speed)
                self.dog.wait_all_done()
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
        text = msg.data.lower().strip()
        
        # Filter garbage
        if len(text) < 2:
            return
        
        false_positives = ['ah', 'uh', 'um', 'oh', 'by', 'do', 'go', 'to', 'be', 'me']
        if text in false_positives:
            return
        
        with self.command_lock:
            if self.command_active:
                return
        
        self.get_logger().info(f"🎤 Voice command: '{text}'")
        self.voice_command_waiting = True
        
        # Process command
        if 'sit' in text:
            self.get_logger().info("📢 SIT")
            self.execute_movement('sit')
            self.speak("Sitting down")
            self.state = RobotState.INTERACTING
            threading.Timer(3.0, self.return_to_wandering).start()
            
        elif 'stand' in text:
            self.get_logger().info("📢 STAND")
            self.execute_movement('stand')
            self.speak("Standing up")
            self.state = RobotState.INTERACTING
            threading.Timer(2.0, self.return_to_wandering).start()
            
        elif 'walk' in text or 'forward' in text:
            self.get_logger().info("📢 WALK")
            self.execute_movement('forward', step_count=6, speed=80)
            
        elif 'back' in text:
            self.get_logger().info("📢 BACK")
            self.execute_movement('backward', step_count=4, speed=80)
            
        elif 'stop' in text and len(text) < 10:
            self.get_logger().info("📢 STOP")
            self.execute_movement('stop')
            self.speak("Stopping")
            self.state = RobotState.INTERACTING
            threading.Timer(2.0, self.return_to_wandering).start()
            
        elif 'left' in text and len(text) < 10:
            self.get_logger().info("📢 TURN LEFT")
            self.execute_movement('turn_left', step_count=4, speed=70)
            
        elif 'right' in text and len(text) < 10:
            self.get_logger().info("📢 TURN RIGHT")
            self.execute_movement('turn_right', step_count=4, speed=70)
        
        self.voice_command_waiting = False
    
    def speak(self, text, use_emotion=False):
        """Send speech command to TTS node"""
        cmd_str = f"{text}:{1 if use_emotion else 0}"
        msg = String()
        msg.data = cmd_str
        self.speak_pub.publish(msg)
        self.get_logger().info(f"🔊 Speaking: {text}")
    
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
    
    def handle_touch_event(self, sensor_id):
        """React to touch sensor events"""
        if self.state != RobotState.WANDERING:
            return
        
        self.state = RobotState.INTERACTING
        self.speak("That tickles! Hehe!", use_emotion=True)
        self.execute_movement("shake_head", step_count=3, speed=80)
        threading.Timer(5.0, self.return_to_wandering).start()
    
    def handle_face_detection(self):
        """React to detected faces"""
        if self.face_count > 0 and self.state == RobotState.WANDERING:
            self.state = RobotState.INTERACTING
            self.get_logger().info(f"Detected {self.face_count} face(s)")
            self.execute_movement("stop")
            self.speak(f"Hi there! I see {self.face_count} of you!", use_emotion=True)
            self.execute_movement("wag_tail", step_count=5, speed=90)
            threading.Timer(3.0, self.return_to_wandering).start()
    
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
        
        self.get_logger().info(f"Smart turning {direction}")
        self.execute_movement(f"turn_{direction}", step_count=5, speed=TURN_SPEED)
        self.turn_history.append(direction)
        return direction
    
    def obstacle_avoidance(self):
        """Handle obstacle avoidance behavior"""
        with self.command_lock:
            if self.command_active or self.emergency_stop:
                return
        
        self.state = RobotState.AVOIDING
        self.get_logger().info(f"🚧 Obstacle detected! Distance: {self.current_distance:.1f} cm")
        
        self.execute_movement("stop")
        time.sleep(0.3)
        
        self.execute_movement("backward", step_count=5, speed=FORWARD_SPEED)
        time.sleep(BACKWARD_TIME)
        
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
            self.speak(emotion_sounds[emotion], use_emotion=True)
            self.emotion = emotion
            
            if emotion == Emotion.HAPPY:
                self.execute_movement("wag_tail", step_count=3, speed=90)
            elif emotion == Emotion.CURIOUS:
                self.execute_movement("head_tilt", step_count=0, speed=50)
            elif emotion == Emotion.STARTLED:
                self.execute_movement("startle", step_count=0, speed=80)
    
    def random_personality_action(self):
        """Perform random action for personality"""
        with self.command_lock:
            if self.command_active:
                return
        
        actions = [
            ("scratch", 3, 70),
            ("shake_head", 3, 80),
            ("wag_tail", 5, 90),
            ("look_around", 0, 50),
        ]
        
        action = random.choice(actions)
        self.get_logger().info(f"🎭 Random personality: {action[0]}")
        self.execute_movement(action[0], action[1], action[2])
    
    def return_to_wandering(self):
        """Return to wandering state after interaction"""
        if not self.emergency_stop:
            self.state = RobotState.WANDERING
            self.get_logger().info("Returning to wandering mode")
    
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
                
                # Check if voice command is active
                with self.command_lock:
                    if self.command_active:
                        time.sleep(0.1)
                        continue
                
                # Different behaviors based on state
                if self.state == RobotState.WANDERING:
                    # Check for obstacles
                    if self.current_distance and self.current_distance < OBSTACLE_DISTANCE_CM:
                        self.obstacle_avoidance()
                    else:
                        # Move forward normally
                        self.execute_movement("forward", step_count=2, speed=FORWARD_SPEED)
                        time.sleep(0.5)
                        
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
                    
                    time.sleep(0.3)
                    
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
