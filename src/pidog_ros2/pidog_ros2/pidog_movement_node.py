#!/usr/bin/env python3
"""
ROS 2 Movement Control Node for Pi Dog
Handles all Pi Dog leg, head, and body movements
Uses standard ROS 2 message types (no custom imports required)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Standard ROS 2 imports instead of custom ones
from std_srvs.srv import Trigger
from std_msgs.msg import String

import sys
import os
import time
import threading

# Try to import PiDog library
try:
    # Add path for PiDog library if needed
    sys.path.append('/usr/local/lib/python3.10/dist-packages')
    from pidog import Pidog
    PIDOG_AVAILABLE = True
except ImportError:
    PIDOG_AVAILABLE = False
    print("Warning: PiDog library not available. Running in simulation mode.")

# Available actions from PiDog
AVAILABLE_ACTIONS = [
    'sit', 'stand', 'forward', 'backward', 'turn_left', 'turn_right',
    'stretch', 'push_up', 'hand_shake', 'scratch', 'high_five',
    'shake_head', 'wag_tail', 'head_tilt', 'look_around', 'bark',
    'howling', 'attack_posture', 'lick_hand', 'waiting', 'feet_shake',
    'sit_2_stand', 'relax_neck', 'nod', 'think', 'recall',
    'head_down_left', 'head_down_right', 'fluster', 'alert', 'surprise',
    'trot', 'body_twisting', 'pant', 'startle', 'stop'
]


class PiDogMovementNode(Node):
    def __init__(self):
        super().__init__('pidog_movement_node')
        
        # Initialize PiDog
        self.dog = None
        self.is_moving = False
        self.current_action = None
        
        if PIDOG_AVAILABLE:
            try:
                self.dog = Pidog()
                self.get_logger().info("PiDog hardware initialized successfully")
                
                # Initialize servos to safe position
                self.dog.do_action('sit', speed=50)
                self.dog.wait_all_done()
                
            except Exception as e:
                self.get_logger().error(f"Failed to initialize PiDog: {e}")
                self.dog = None
        else:
            self.get_logger().warning("Running in simulation mode - no hardware control")
        
        # ROS 2 Services using standard Trigger type
        # Each command gets its own service
        self.sit_srv = self.create_service(Trigger, 'sit', self.make_service_callback('sit'))
        self.stand_srv = self.create_service(Trigger, 'stand', self.make_service_callback('stand'))
        self.forward_srv = self.create_service(Trigger, 'forward', self.make_service_callback('forward'))
        self.backward_srv = self.create_service(Trigger, 'backward', self.make_service_callback('backward'))
        self.turn_left_srv = self.create_service(Trigger, 'turn_left', self.make_service_callback('turn_left'))
        self.turn_right_srv = self.create_service(Trigger, 'turn_right', self.make_service_callback('turn_right'))
        self.stop_srv = self.create_service(Trigger, 'stop', self.make_service_callback('stop'))
        self.stretch_srv = self.create_service(Trigger, 'stretch', self.make_service_callback('stretch'))
        self.push_up_srv = self.create_service(Trigger, 'push_up', self.make_service_callback('push_up'))
        self.hand_shake_srv = self.create_service(Trigger, 'hand_shake', self.make_service_callback('hand_shake'))
        self.scratch_srv = self.create_service(Trigger, 'scratch', self.make_service_callback('scratch'))
        self.high_five_srv = self.create_service(Trigger, 'high_five', self.make_service_callback('high_five'))
        self.shake_head_srv = self.create_service(Trigger, 'shake_head', self.make_service_callback('shake_head'))
        self.bark_srv = self.create_service(Trigger, 'bark', self.make_service_callback('bark'))
        self.howling_srv = self.create_service(Trigger, 'howling', self.make_service_callback('howling'))
        
        # Command subscriber for string commands (alternative interface)
        self.cmd_sub = self.create_subscription(
            String,
            'command',
            self.command_callback,
            10
        )
        
        # Status publisher using standard String message
        self.status_pub = self.create_publisher(
            String,
            'movement_status',
            10
        )
        
        self.get_logger().info("PiDog Movement Node Ready")
        self.get_logger().info("Available services: sit, stand, forward, backward, turn_left, turn_right, stop, stretch, push_up, hand_shake, scratch, high_five, shake_head, bark, howling")
        self.get_logger().info("Or publish to /command topic with string commands")
    
    def make_service_callback(self, command):
        """Factory method to create service callbacks for different commands"""
        def callback(request, response):
            self.get_logger().info(f"{command} command received via service")
            
            # Execute the command
            success = self.execute_movement(command, step_count=5, speed=70, wait=True)
            
            response.success = success
            response.message = f"{command} {'completed' if success else 'failed'}"
            return response
        return callback
    
    def command_callback(self, msg):
        """Handle string commands from topic"""
        cmd = msg.data.lower().strip()
        self.get_logger().info(f"Command via topic: {cmd}")
        
        # Parse command (could include parameters like "forward 10" for step count)
        parts = cmd.split()
        action = parts[0]
        step_count = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 5
        
        if action in AVAILABLE_ACTIONS or action == 'stop':
            self.execute_movement(action, step_count=step_count, speed=70, wait=False)
        else:
            self.get_logger().warning(f"Unknown command: {cmd}")
    
    def execute_movement(self, command, step_count=5, speed=70, wait=True):
        """Execute the actual movement on PiDog"""
        if self.dog is None:
            # Simulation mode
            self.get_logger().info(f"[SIM] Executing: {command} (steps={step_count}, speed={speed})")
            if wait:
                time.sleep(0.5)
            
            # Publish status
            status_msg = String()
            status_msg.data = f"action:{command}:success:sim"
            self.status_pub.publish(status_msg)
            return True
        
        try:
            # Stop command
            if command == 'stop':
                self.dog.body_stop()
                self.dog.wait_all_done()
                status_msg = String()
                status_msg.data = "action:stop:success"
                self.status_pub.publish(status_msg)
                return True
            
            # Custom action for head movement
            if command == 'head_tilt':
                self.dog.head_move([(0, 0, 15)], immediately=True, speed=speed)
                if wait:
                    time.sleep(0.5)
                    self.dog.head_move([(0, 0, 0)], immediately=True, speed=speed)
                status_msg = String()
                status_msg.data = "action:head_tilt:success"
                self.status_pub.publish(status_msg)
                return True
            
            if command == 'look_around':
                angles = [(-30, 0, 0), (30, 0, 0), (0, 0, 0)]
                for yaw, pitch, roll in angles:
                    self.dog.head_move([(yaw, pitch, roll)], immediately=True, speed=speed)
                    time.sleep(0.3)
                status_msg = String()
                status_msg.data = "action:look_around:success"
                self.status_pub.publish(status_msg)
                return True
            
            if command == 'wag_tail':
                # Tail wagging - use body twisting
                steps = step_count if step_count > 0 else 5
                for _ in range(steps):
                    self.dog.do_action('body_twisting', speed=speed)
                    self.dog.wait_all_done()
                    time.sleep(0.2)
                status_msg = String()
                status_msg.data = "action:wag_tail:success"
                self.status_pub.publish(status_msg)
                return True
            
            if command == 'startle':
                # Startle response - quick backwards and shake
                self.dog.do_action('backward', step_count=3, speed=90)
                self.dog.wait_all_done()
                self.dog.do_action('shake_head', step_count=2, speed=90)
                status_msg = String()
                status_msg.data = "action:startle:success"
                self.status_pub.publish(status_msg)
                return True
            
            # Standard actions from preset_actions
            if hasattr(self.dog, 'do_action'):
                # Use preset actions if available
                try:
                    from pidog.preset_actions import (
                        hand_shake, scratch, high_five, pant, body_twisting,
                        bark_action, shake_head, shake_head_smooth, howling,
                        attack_posture, lick_hand, waiting, feet_shake,
                        sit_2_stand, relax_neck, nod, think, recall,
                        head_down_left, head_down_right, fluster, alert, surprise
                    )
                    
                    action_map = {
                        'hand_shake': lambda: hand_shake(self.dog),
                        'scratch': lambda: scratch(self.dog),
                        'high_five': lambda: high_five(self.dog),
                        'howling': lambda: howling(self.dog),
                        'bark': lambda: bark_action(self.dog)
                    }
                    
                    if command in action_map:
                        action_map[command]()
                        if wait:
                            self.dog.wait_all_done()
                        status_msg = String()
                        status_msg.data = f"action:{command}:success"
                        self.status_pub.publish(status_msg)
                        return True
                except ImportError:
                    pass
                
                # Use do_action for standard actions
                steps = max(1, step_count) if step_count > 0 else 1
                self.dog.do_action(command, step_count=steps, speed=speed)
                
                if wait:
                    self.dog.wait_all_done()
                
                status_msg = String()
                status_msg.data = f"action:{command}:success"
                self.status_pub.publish(status_msg)
                return True
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"Movement execution error: {e}")
            status_msg = String()
            status_msg.data = f"action:{command}:error:{str(e)}"
            self.status_pub.publish(status_msg)
            return False
    
    def shutdown(self):
        """Safe shutdown of PiDog"""
        if self.dog:
            self.get_logger().info("Shutting down PiDog movement...")
            try:
                self.dog.body_stop()
                self.dog.do_action('sit', speed=50)
                self.dog.wait_all_done()
                time.sleep(0.5)
                self.dog.close()
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = PiDogMovementNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Movement node interrupted")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
