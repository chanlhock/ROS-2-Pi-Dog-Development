#!/usr/bin/env python3
"""
ROS 2 Main Autonomous Node for Pi Dog
Coordinates all submodules for autonomous behavior
Uses standard ROS 2 message types (no custom imports required)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy , ReliabilityPolicy, HistoryPolicy

# Standard ROS 2 imports instead of custom ones
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger

import threading
import time
import random
from collections import deque
from enum import Enum

# Constants
OBSTACLE_DISTANCE_CM = 30
FORWARD_SPEED = 98
TURN_SPEED = 98
BACKWARD_TIME = 1.0
TURN_TIME = 0.6
MSE_THRESHOLD = 20


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
        
        # State management
        self.state = RobotState.WANDERING
        self.emotion = Emotion.HAPPY
        self.last_emotion_time = time.time()
        self.emotion_interval = 30
        
        # Sensor data cache (using standard types)
        self.current_distance = 999.0
        self.current_imu = None  # Will be parsed from String
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
        
        # After existing parameters, add:
        self.declare_parameter('enable_head_scanning', False)
        self.declare_parameter('enable_sound_turning', True)
        self.declare_parameter('enable_face_interaction', True)
        self.declare_parameter('personality_actions', True)

        self.enable_head_scanning = self.get_parameter('enable_head_scanning').value
        self.enable_sound_turning = self.get_parameter('enable_sound_turning').value
        self.enable_face_interaction = self.get_parameter('enable_face_interaction').value
        self.personality_actions = self.get_parameter('personality_actions').value

        # ROS 2 Publishers
        #qos_profile = QoSProfile(
        #    reliability=ReliabilityPolicy.RELIABLE,
        #    history=HistoryPolicy.KEEP_LAST,
        #    depth=10
        #)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Change from RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=10
       )
        
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Change back to RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=10
      )
        
        # Status publisher using String
        self.status_pub = self.create_publisher(
            String,
            'status',
            qos_profile
        )
        
        # ROS 2 Subscribers with separate callback groups
        self.dist_cb_group = MutuallyExclusiveCallbackGroup()
        self.imu_cb_group = MutuallyExclusiveCallbackGroup()
        self.touch_cb_group = MutuallyExclusiveCallbackGroup()
        self.sound_cb_group = MutuallyExclusiveCallbackGroup()
        self.face_cb_group = MutuallyExclusiveCallbackGroup()
        self.voice_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Distance subscriber (Float32)
        self.distance_sub = self.create_subscription(
            Float32,
            'distance',
            self.distance_callback,
            qos_profile,
            callback_group=self.dist_cb_group
        )
        
        # IMU subscriber (String with JSON-like format)
        self.imu_sub = self.create_subscription(
            String,
            'imu',
            self.imu_callback,
            qos_profile,
            callback_group=self.imu_cb_group
        )
        
        # Touch subscriber (String: "touched:sensor_id")
        self.touch_sub = self.create_subscription(
            String,
            'touch',
            self.touch_callback,
            qos_profile,
            callback_group=self.touch_cb_group
        )
        
        # Sound direction subscriber (String: "direction:angle:confidence")
        self.sound_dir_sub = self.create_subscription(
            String,
            'sound_direction',
            self.sound_direction_callback,
            qos_profile,
            callback_group=self.sound_cb_group
        )
        
        # Face detection subscriber (String: "count:x1,y1,w1,h1:x2,y2,...")
        self.face_sub = self.create_subscription(
            String,
            'face_detection',
            self.face_callback,
            qos_profile,
            callback_group=self.face_cb_group
        )
        
        # Voice command subscriber
        self.voice_command_sub = self.create_subscription(
            String,
            'voice_command',
            self.voice_command_callback,
            qos_profile,
            callback_group=self.voice_cb_group
        )
        
        # Movement command publisher (instead of service)
        self.move_pub = self.create_publisher(
            String,
            'command',  # Change from 'movement_command' to 'command'
            qos_profile_reliable
        )
        
        # Speech command publisher
        #self.speak_pub = self.create_publisher(
        #    String,
        #    'speak_command',
        #    qos_profile
        #)
        
        # To (add both publishers):
        self.speak_pub = self.create_publisher(
            String,
            'speak_text',  # Match the TTS node's topic
            qos_profile
        )

        # Also keep speak_command for compatibility
        self.speak_command_pub = self.create_publisher(
            String,
            'speak_command',
            qos_profile
       )

        # Start autonomous behavior thread
        self.autonomous_thread = threading.Thread(target=self.autonomous_behavior_loop, daemon=True)
        self.autonomous_thread.start()
        
        # Start periodic status publisher
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("ROS 2 Autonomous Pi Dog Node Started")
        self.get_logger().info("Waiting for sensor data...")
    
    def distance_callback(self, msg):
        """Handle distance sensor data (Float32)"""
        self.current_distance = msg.data
        self.get_logger().debug(f"Distance updated: {self.current_distance:.1f} cm")
    
    def imu_callback(self, msg):
        """Handle IMU data (String format: "roll,pitch,yaw,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z")"""
        try:
            parts = msg.data.split(',')
            if len(parts) >= 9:
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
            self.get_logger().debug(f"IMU parse error: {e}")
    
    def touch_callback(self, msg):
        """Handle touch sensor data (String format: "touched:sensor_id:confidence")"""
        try:
            parts = msg.data.split(':')
            if len(parts) >= 2:
                touched = parts[0] == 'touched' or parts[0] == 'True'
                sensor_id = int(parts[1]) if len(parts) > 1 else 0
                confidence = float(parts[2]) if len(parts) > 2 else 1.0
                
                self.last_touch = touched
                self.last_touch_sensor = sensor_id
                
                if touched:
                    self.get_logger().info(f"Touch detected on sensor {sensor_id} (confidence: {confidence})")
                    self.handle_touch_event(sensor_id)
        except Exception as e:
            self.get_logger().debug(f"Touch parse error: {e}")
    
    def sound_direction_callback(self, msg):
        """Handle sound direction data (String format: "angle:direction:confidence")"""
        try:
            parts = msg.data.split(':')
            if len(parts) >= 2:
                self.sound_direction_angle = float(parts[0])
                self.sound_direction_text = parts[1] if len(parts) > 1 else "unknown"
                confidence = float(parts[2]) if len(parts) > 2 else 0.5
                
                if self.sound_direction_angle >= 0 and self.state == RobotState.WANDERING:
                    self.handle_sound_direction()
        except Exception as e:
            self.get_logger().debug(f"Sound direction parse error: {e}")
    
    def face_callback(self, msg):
        """Handle face detection data (String format: "count:x1,y1,w1,h1:x2,y2,w2,h2...")"""
        try:
            parts = msg.data.split(':')
            if len(parts) >= 1:
                self.face_count = int(parts[0])
                
                if self.face_count > 0 and self.state == RobotState.WANDERING:
                    self.handle_face_detection()
        except Exception as e:
            self.get_logger().debug(f"Face detection parse error: {e}")
    
    #def voice_command_callback(self, msg):
    #    """Handle voice commands"""
    #    self.get_logger().info(f"Voice command received: {msg.data}")
    #    self.handle_voice_command(msg.data)
    
    # In voice_command_callback, add:
    def voice_command_callback(self, msg):
        command = msg.data.lower().strip()
    
        # List of valid commands
        valid_commands = ['sit', 'stand', 'walk', 'forward', 'back', 'backward', 
                          'left', 'right', 'stop', 'resume', 'shutdown', 'sleep',
                          'stretch', 'hand shake', 'high five', 'scratch', 'howl']
    
        # Check if command matches any valid command
        matched = False
        matched_command = None
        for valid in valid_commands:
            if valid in command:
                matched_command = valid
                matched = True
                break
        # Also check for single words that are valid
        simple_commands = ['sit', 'stand', 'walk', 'stop', 'howl']
        if command in simple_commands:
            matched_command = command
            matched = True

        if matched and len(command) < 20 and len(command) > 2:  # Ignore long false positives
            self.get_logger().info(f"Voice command received: {matched_command}")
            self.handle_voice_command(matched_command)
        elif len(command) > 2:
            self.get_logger().debug(f"Ignoring unrecognized command: {command}")

    def publish_status(self):
        """Publish robot status periodically"""
        status_str = f"state:{self.state.value}:emotion:{self.emotion.value}:distance:{self.current_distance:.1f}:emergency:{self.emergency_stop}"
        status_msg = String()
        status_msg.data = status_str
        self.status_pub.publish(status_msg)
    
    def handle_touch_event(self, sensor_id):
        """React to touch sensor events"""
        self.state = RobotState.INTERACTING
        
        # Play happy sound and react
        self.speak("That tickles! Hehe!", use_emotion=True)
        
        # Perform a happy reaction
        self.send_move_command("shake_head", step_count=3, speed=80)
        
        # Return to wandering after a delay
        threading.Timer(5.0, self.return_to_wandering).start()
    
    def handle_sound_direction(self):
        """Turn towards sound source"""
        self.get_logger().info(f"Sound from {self.sound_direction_text} at {self.sound_direction_angle}°")
        
        # Turn towards sound
        if self.sound_direction_text == "left" and self.sound_direction_angle > 20:
            self.send_move_command("turn_left", step_count=3, speed=70)
        elif self.sound_direction_text == "right" and self.sound_direction_angle > 20:
            self.send_move_command("turn_right", step_count=3, speed=70)
        elif self.sound_direction_text == "back":
            # Turn around completely
            self.send_move_command("turn_left", step_count=10, speed=80)
        
        # Look curious
        self.speak("What was that?")
    
    def handle_face_detection(self):
        """React to detected faces"""
        if self.face_count > 0 and self.state == RobotState.WANDERING:
            self.get_logger().info(f"Detected {self.face_count} face(s)")
            
            # If face is detected, stop and be happy
            self.send_move_command("stop", step_count=0, speed=0)
            self.speak(f"Hi there! I see {self.face_count} of you!", use_emotion=True)
            self.send_move_command("wag_tail", step_count=5, speed=90)
            
            # Return to wandering after interaction
            threading.Timer(3.0, self.return_to_wandering).start()
    
    def handle_voice_command(self, command):
        """Process voice commands from STT node"""
        self.voice_command_waiting = True
        cmd_lower = command.lower()
        
        # Emergency stop has highest priority
        if "stop" in cmd_lower and "emergency" in cmd_lower:
            self.emergency_stop = True
            self.state = RobotState.EMERGENCY_STOP
            self.send_move_command("stop", step_count=0, speed=0)
            self.speak("Emergency stop activated")
            self.voice_command_waiting = False
            return
        
        if "resume" in cmd_lower or "continue" in cmd_lower:
            self.emergency_stop = False
            self.state = RobotState.WANDERING
            self.speak("Resuming wandering")
            self.voice_command_waiting = False
            return
        
        if "shutdown" in cmd_lower or "sleep" in cmd_lower:
            self.speak("Going to sleep now. Goodbye!")
            self.send_move_command("sit", step_count=0, speed=50)
            self.state = RobotState.SLEEPING
            self.voice_command_waiting = False
            return
        
        # Movement commands
        if any(word in cmd_lower for word in ["sit", "sit down"]):
            self.send_move_command("sit", step_count=0, speed=50)
            self.speak("Sitting down")
            
        elif any(word in cmd_lower for word in ["stand", "stand up"]):
            self.send_move_command("stand", step_count=0, speed=70)
            self.speak("Standing up")
            
        elif any(word in cmd_lower for word in ["walk", "walking", "forward"]):
            self.send_move_command("forward", step_count=8, speed=80)
            
        elif any(word in cmd_lower for word in ["back", "backward"]):
            self.send_move_command("backward", step_count=5, speed=80)
            
        elif "left" in cmd_lower:
            self.send_move_command("turn_left", step_count=4, speed=80)
            
        elif "right" in cmd_lower:
            self.send_move_command("turn_right", step_count=4, speed=80)
            
        elif "stretch" in cmd_lower:
            self.send_move_command("stretch", step_count=0, speed=70)
            
        elif "hand" in cmd_lower or "shake" in cmd_lower:
            self.send_move_command("hand_shake", step_count=0, speed=70)
            
        elif "high five" in cmd_lower:
            self.send_move_command("high_five", step_count=0, speed=70)
            
        elif "scratch" in cmd_lower:
            self.send_move_command("scratch", step_count=0, speed=70)
            
        elif "howl" in cmd_lower:
            self.speak("Wooooo! Awooo!", use_emotion=True)
        
        self.voice_command_waiting = False
    
    #def send_move_command(self, command, step_count=0, speed=70):
    #    """Send movement command to movement node via topic"""
    #    # Format: "command:step_count:speed"
    #    cmd_str = f"{command}:{step_count}:{speed}"
    #    msg = String()
    #    msg.data = cmd_str
    #    self.move_pub.publish(msg)
    #    self.get_logger().info(f"Sent move command: {command}")
    
    # Replace lines 256-264 (send_move_command method):
    #def send_move_command(self, command, step_count=0, speed=70):
    #    """Send movement command to movement node via topic"""
    #    # The movement node expects format: "command:step_count:speed"
    #    # But it also accepts plain strings without colon
    #    if step_count > 0:
    #        cmd_str = f"{command}:{step_count}:{speed}"
    #    else:
    #        cmd_str = command  # For commands like 'sit', 'stand' that don't need step_count
   # 
    #    msg = String()
    #    msg.data = cmd_str
    #    self.move_pub.publish(msg)
    #    self.get_logger().info(f"Sent move command: {command}")

    def send_move_command(self, command, step_count=0, speed=70):
        """Send movement command to movement node via topic"""
        # The movement node expects different formats for different commands
        # For commands that are just strings (sit, stand, stop)
        if command in ['sit', 'stand', 'stop', 'stretch', 'push_up', 
                       'hand_shake', 'scratch', 'high_five', 'shake_head', 
                       'bark', 'howling', 'startle', 'wag_tail', 'look_around']:
            cmd_str = command
        else:
            # For movement commands that need step count
            cmd_str = f"{command}:{step_count}:{speed}"
    
        msg = String()
        msg.data = cmd_str
        self.move_pub.publish(msg)
        self.get_logger().info(f"Sent move command: {cmd_str}")

    def speak(self, text, use_emotion=False):
        """Send speech command to TTS node via topic"""
        # Format: "text:use_emotion"
        cmd_str = f"{text}:{1 if use_emotion else 0}"
        msg = String()
        msg.data = cmd_str
        self.speak_pub.publish(msg)
        self.get_logger().info(f"Speaking: {text}")
    
    def get_distance(self):
        """Get current distance from sensor"""
        return self.current_distance if self.current_distance < 999 else None
    
    # In ros2_autonomous_pidog.py:
    #def get_distance(self):
        """Get current distance from sensor"""
    #    if self.distance_node_available:
    #        return self.current_distance if self.current_distance < 999 else None
    #    else:
    #        # Simulate distance for testing
    #        return random.uniform(100, 300)  # Random distance

    def smart_turn(self):
        """Intelligent turning based on turn history"""
        # Alternate turns if we've been turning same direction too much
        if self.turn_history.count("left") >= 3:
            direction = "right"
        elif self.turn_history.count("right") >= 3:
            direction = "left"
        else:
            # Use sound direction if available for smarter turning
            if self.sound_direction_angle > 0:
                direction = "left" if self.sound_direction_angle < 180 else "right"
            else:
                direction = "left" if random.random() > 0.5 else "right"
        
        self.get_logger().info(f"Smart turning {direction}")
        self.send_move_command(f"turn_{direction}", step_count=5, speed=TURN_SPEED)
        self.turn_history.append(direction)
        time.sleep(TURN_TIME)
    
    def obstacle_avoidance(self):
        """Handle obstacle avoidance behavior"""
        if self.emergency_stop:
            return
        
        self.state = RobotState.AVOIDING
        self.get_logger().info("Obstacle detected, avoiding...")
        
        # Stop moving
        self.send_move_command("stop", step_count=0, speed=0)
        time.sleep(0.3)
        
        # Back up
        self.send_move_command("backward", step_count=5, speed=FORWARD_SPEED)
        time.sleep(BACKWARD_TIME)
        
        # Turn away
        self.smart_turn()
        
        # Express emotion
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
            
            # Quick movement for emotion
            if emotion == Emotion.HAPPY:
                self.send_move_command("wag_tail", step_count=3, speed=90)
            elif emotion == Emotion.CURIOUS:
                self.send_move_command("head_tilt", step_count=0, speed=50)
            elif emotion == Emotion.STARTLED:
                self.send_move_command("startle", step_count=0, speed=80)
    
    def return_to_wandering(self):
        """Return to wandering state after interaction"""
        if not self.emergency_stop:
            self.state = RobotState.WANDERING
            self.get_logger().info("Returning to wandering mode")
    
    def autonomous_behavior_loop(self):
        """Main autonomous behavior loop"""
        self.get_logger().info("Starting autonomous behavior loop")
        
        # Stand up initially
        self.send_move_command("stand", step_count=0, speed=70)
        time.sleep(2)
        
        while rclpy.ok():
            try:
                # Check for emergency stop
                if self.emergency_stop:
                    time.sleep(0.5)
                    continue
                
                # Different behaviors based on state
                if self.state == RobotState.WANDERING:
                    # Check for obstacles
                    if self.current_distance and self.current_distance < OBSTACLE_DISTANCE_CM:
                        self.obstacle_avoidance()
                    else:
                        # Move forward normally
                        self.send_move_command("forward", step_count=2, speed=FORWARD_SPEED)
                        time.sleep(0.5)  # Add delay instead of 0.3 to give time to complete movement

                        # Random personality actions occasionally
                        if random.random() < 0.05:  # 5% chance
                            self.random_personality_action()
                        
                        # Periodic emotions
                        now = time.time()
                        if now - self.last_emotion_time > self.emotion_interval:
                            emotions = list(Emotion)
                            random_emotion = random.choice(emotions)
                            self.emotion = random_emotion
                            self.play_emotion(random_emotion)
                            self.last_emotion_time = now
                    
                    time.sleep(0.3)
                    
                elif self.state == RobotState.SLEEPING:
                    # Do nothing, just sleep
                    time.sleep(1.0)
                    
                else:
                    # Other states, just wait
                    time.sleep(0.1)
                    
            except Exception as e:
                self.get_logger().error(f"Error in autonomous behavior: {e}")
                time.sleep(1.0)
    
    def random_personality_action(self):
        """Perform random action for personality"""
        actions = [
            ("scratch", 3, 70),
            ("shake_head", 3, 80),
            ("wag_tail", 5, 90),
            ("look_around", 0, 50),
            ("sit", 0, 50),
            ("stand", 0, 70)
        ]
        
        action = random.choice(actions)
        self.get_logger().info(f"Random personality: {action[0]}")
        self.send_move_command(action[0], action[1], action[2])


def main(args=None):
    rclpy.init(args=args)
    
    node = Ros2AutonomousPiDog()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down autonomous Pi Dog node")
    finally:
        node.destroy_node()
        if rclpy.ok():  # Check if already shutdown
            rclpy.shutdown()


if __name__ == '__main__':
    main()
