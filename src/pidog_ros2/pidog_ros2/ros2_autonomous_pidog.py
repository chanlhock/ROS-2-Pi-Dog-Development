#!/usr/bin/env python3
"""
ROS 2 Main Autonomous Node for Pi Dog
THIS IS THE ONLY NODE THAT INITIALIZES PIDOG HARDWARE
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

import threading
import time
import random
from collections import deque
from enum import Enum

# Import PiDog Manager (local import - same directory)
from .pidog_manager import get_pidog_manager

# Constants
OBSTACLE_DISTANCE_CM = 30
FORWARD_SPEED = 98
TURN_SPEED = 98
BACKWARD_TIME = 1.0
TURN_TIME = 0.6


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
        # INITIALIZE PIDOG HARDWARE (ONLY HERE!)
        # ============================================================
        self.get_logger().info("=" * 60)
        self.get_logger().info("Initializing PiDog hardware...")
        
        self.pidog_manager = get_pidog_manager()
        
        if self.pidog_manager.initialize(disable_sensors=True):
            self.dog = self.pidog_manager.get_pidog()
            self.get_logger().info("✓ PiDog hardware available - movement enabled")
            
            # Test hardware
            try:
                self.get_logger().info("Testing hardware (sit/stand)...")
                self.dog.do_action('sit', speed=50)
                self.dog.wait_all_done()
                time.sleep(1)
                self.dog.do_action('stand', speed=50)
                self.dog.wait_all_done()
                self.get_logger().info("✓ Hardware test successful!")
            except Exception as e:
                self.get_logger().error(f"Hardware test failed: {e}")
                return
        else:
            self.dog = None
            self.get_logger().error("✗ PiDog hardware NOT available!")
            self.get_logger().error("Make sure PiDog is connected and powered on.")
            return
        
        # ============================================================
        # STATE MANAGEMENT
        # ============================================================
        self.state = RobotState.WANDERING
        self.emotion = Emotion.HAPPY
        self.last_emotion_time = time.time()
        self.emotion_interval = 30
        
        # Sensor data
        self.current_distance = 999.0
        self.sound_direction_angle = -1.0
        self.sound_direction_text = "unknown"
        self.face_count = 0
        self.last_touch = False
        
        # Behavior tracking
        self.turn_history = deque(maxlen=5)
        self.emergency_stop = False
        self.voice_command_waiting = False
        
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
        
        # Subscribers
        self.voice_sub = self.create_subscription(String, 'voice_command', self.voice_callback, qos_best)
        self.cmd_sub = self.create_subscription(String, 'command', self.command_callback, qos_rel)
        
        # ============================================================
        # START THREADS
        # ============================================================
        self.sensor_thread = threading.Thread(target=self.sensor_loop, daemon=True)
        self.sensor_thread.start()
        
        self.behavior_thread = threading.Thread(target=self.behavior_loop, daemon=True)
        self.behavior_thread.start()
        
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("ROS 2 Autonomous PiDog Node Ready!")
        self.get_logger().info("Features: Autonomous Wandering | Obstacle Avoidance | Voice Commands")
        self.get_logger().info("Say: sit, stand, walk, stop, turn left, turn right")
        self.get_logger().info("=" * 60)
    
    # ============================================================
    # SENSOR LOOP
    # ============================================================
    def sensor_loop(self):
        """Read sensors directly from hardware"""
        self.get_logger().info("Sensor reading thread started")
        
        while rclpy.ok() and self.dog:
            try:
                # Read distance
                if hasattr(self.dog, 'ultrasonic') and hasattr(self.dog.ultrasonic, 'read_distance'):
                    dist = self.dog.ultrasonic.read_distance()
                    if dist and 2 <= dist <= 400:
                        self.current_distance = float(dist)
                        self.distance_pub.publish(Float32(data=self.current_distance))
                
                # Read sound direction
                if hasattr(self.dog, 'ears') and hasattr(self.dog.ears, 'isdetected'):
                    if self.dog.ears.isdetected() and hasattr(self.dog.ears, 'read'):
                        angle = self.dog.ears.read()
                        if angle is not None and 0 <= angle <= 360:
                            self.sound_direction_angle = angle
                            self.sound_pub.publish(String(data=f"{angle:.1f}:detected"))
                
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().debug(f"Sensor error: {e}")
                time.sleep(0.5)
    
    # ============================================================
    # MOVEMENT EXECUTION
    # ============================================================
    def move(self, command, steps=5, speed=70):
        """Execute movement directly on hardware"""
        if self.dog is None:
            return
        
        try:
            self.get_logger().info(f"➡️ {command}")
            
            if command == 'stop':
                self.dog.body_stop()
                self.dog.wait_all_done()
            elif command in ['sit', 'stand']:
                self.dog.do_action(command, speed=speed)
                self.dog.wait_all_done()
            else:
                self.dog.do_action(command, step_count=max(1, steps), speed=speed)
                self.dog.wait_all_done()
        except Exception as e:
            self.get_logger().error(f"Movement error: {e}")
    
    # ============================================================
    # VOICE COMMAND HANDLING
    # ============================================================
    def voice_callback(self, msg):
        """Handle voice commands"""
        text = msg.data.lower().strip()
        self.get_logger().info(f"🎤 Voice: '{text}'")
        
        # Command mapping
        if 'sit' in text or text == 'sit':
            self.move('sit')
            self.speak("Sitting down")
        elif 'stand' in text or text == 'stand':
            self.move('stand')
            self.speak("Standing up")
        elif 'walk' in text or text == 'walk' or 'forward' in text:
            self.move('forward', steps=6, speed=80)
        elif 'back' in text:
            self.move('backward', steps=4, speed=80)
        elif 'left' in text:
            self.move('turn_left', steps=4, speed=70)
        elif 'right' in text:
            self.move('turn_right', steps=4, speed=70)
        elif 'stop' in text:
            self.move('stop')
            self.speak("Stopping")
    
    def command_callback(self, msg):
        """Handle external commands"""
        self.get_logger().info(f"Command: {msg.data}")
        parts = msg.data.lower().split(':')
        cmd = parts[0]
        steps = int(parts[1]) if len(parts) > 1 else 5
        speed = int(parts[2]) if len(parts) > 2 else 70
        self.move(cmd, steps, speed)
    
    def speak(self, text):
        """Publish speech"""
        msg = String()
        msg.data = f"{text}:0"
        self.speak_pub.publish(msg)
    
    # ============================================================
    # AUTONOMOUS BEHAVIORS
    # ============================================================
    def publish_status(self):
        """Publish status"""
        status = f"state:{self.state.value}:distance:{self.current_distance:.1f}:emergency:{self.emergency_stop}"
        self.status_pub.publish(String(data=status))
    
    def smart_turn(self):
        """Intelligent turn"""
        if self.turn_history.count("left") >= 3:
            direction = "right"
        elif self.turn_history.count("right") >= 3:
            direction = "left"
        else:
            direction = "left" if random.random() > 0.5 else "right"
        
        self.move(f"turn_{direction}", steps=5, speed=TURN_SPEED)
        self.turn_history.append(direction)
        time.sleep(TURN_TIME)
    
    def obstacle_avoidance(self):
        """Handle obstacle avoidance"""
        if self.emergency_stop:
            return
        
        self.state = RobotState.AVOIDING
        self.get_logger().info("🚧 Obstacle detected!")
        
        self.move("stop")
        time.sleep(0.3)
        
        self.move("backward", steps=5, speed=FORWARD_SPEED)
        time.sleep(BACKWARD_TIME)
        
        self.smart_turn()
        self.state = RobotState.WANDERING
    
    def play_emotion(self, emotion):
        """Express emotion"""
        emotions = {
            Emotion.HAPPY: "Happy bark!",
            Emotion.CURIOUS: "Hmm? What's that?",
            Emotion.STARTLED: "Woof! That scared me!",
            Emotion.BORED: "Sigh... I'm bored",
            Emotion.LONELY: "Awooo... anyone there?"
        }
        if emotion in emotions:
            self.speak(emotions[emotion])
    
    def random_personality(self):
        """Random personality action"""
        actions = ["shake_head", "wag_tail", "look_around", "scratch"]
        action = random.choice(actions)
        self.get_logger().info(f"🎭 Random personality: {action}")
        self.move(action, steps=3, speed=70)
    
    def behavior_loop(self):
        """Main autonomous behavior loop"""
        self.get_logger().info("Behavior loop started")
        time.sleep(2)
        
        self.move("stand")
        time.sleep(1)
        
        while rclpy.ok():
            try:
                if self.emergency_stop:
                    time.sleep(0.5)
                    continue
                
                if self.state == RobotState.WANDERING:
                    if self.current_distance < OBSTACLE_DISTANCE_CM:
                        self.obstacle_avoidance()
                    else:
                        self.move("forward", steps=2, speed=FORWARD_SPEED)
                        time.sleep(0.5)
                        
                        # Random personality (5% chance)
                        if random.random() < 0.05 and self.personality_actions:
                            self.random_personality()
                        
                        # Periodic emotions (every 30 seconds)
                        now = time.time()
                        if now - self.last_emotion_time > self.emotion_interval:
                            emotions = list(Emotion)
                            self.emotion = random.choice(emotions)
                            self.play_emotion(self.emotion)
                            self.last_emotion_time = now
                    
                    time.sleep(0.3)
                else:
                    time.sleep(0.1)
                    
            except Exception as e:
                self.get_logger().error(f"Behavior error: {e}")
                time.sleep(1)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down...")
        if self.dog:
            try:
                self.move("sit")
                self.pidog_manager.shutdown()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = Ros2AutonomousPiDog()
    
    if node.dog is None:
        node.get_logger().error("No hardware - exiting")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
