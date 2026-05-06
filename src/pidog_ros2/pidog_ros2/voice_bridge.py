#!/usr/bin/env python3
"""
Voice Command Bridge - Forwards voice commands to movement topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceBridge(Node):
    def __init__(self):
        super().__init__('voice_bridge')
        self.get_logger().info("Voice Bridge Node Started")
        
        # Publisher to command topic (for movement)
        self.command_pub = self.create_publisher(String, 'command', 10)
        
        # Subscriber to voice commands
        self.voice_sub = self.create_subscription(
            String,
            'voice_command',
            self.voice_callback,
            10
        )
        
        # Command mapping
        self.command_map = {
            'sit': 'sit',
            'sit down': 'sit',
            'stand': 'stand',
            'stand up': 'stand',
            'walk': 'forward',
            'walk forward': 'forward',
            'go forward': 'forward',
            'forward': 'forward',
            'back': 'backward',
            'backward': 'backward',
            'go back': 'backward',
            'stop': 'stop',
            'halt': 'stop',
            'freeze': 'stop',
            'left': 'turn_left',
            'turn left': 'turn_left',
            'right': 'turn_right',
            'turn right': 'turn_right',
            'stretch': 'stretch',
            'hand shake': 'hand_shake',
            'shake hands': 'hand_shake',
            'scratch': 'scratch',
            'high five': 'high_five',
            'high-five': 'high_five',
        }
        
        self.get_logger().info("Voice Bridge Ready - mapping voice commands to actions")
        self.get_logger().info(f"Commands: {list(self.command_map.keys())}")
    
    def voice_callback(self, msg):
        text = msg.data.lower().strip()
        self.get_logger().info(f"📝 Voice received: '{text}'")
        
        # Find matching command
        command = None
        matched_phrase = None
        
        for phrase, cmd in self.command_map.items():
            if phrase in text:
                command = cmd
                matched_phrase = phrase
                break
        
        # Also check exact match for short commands
        if not command and len(text) < 15:
            for phrase, cmd in self.command_map.items():
                if text == phrase:
                    command = cmd
                    matched_phrase = phrase
                    break
        
        if command:
            self.get_logger().info(f"🎯 Matched '{matched_phrase}' -> Executing: {command}")
            cmd_msg = String()
            cmd_msg.data = command
            self.command_pub.publish(cmd_msg)
        else:
            self.get_logger().info(f"❌ No matching command for: '{text}'")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
