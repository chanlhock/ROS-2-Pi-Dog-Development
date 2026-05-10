#!/usr/bin/env python3
"""
ROS 2 Movement Control Node for Pi Dog
NO HARDWARE ACCESS - forwards commands to main node via topics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_srvs.srv import Trigger
from std_msgs.msg import String

import time


class PiDogMovementNode(Node):
    """
    ROS 2 Node for PiDog movement control.
    
    This node does NOT access hardware directly. It forwards movement
    commands to the main autonomous node via the 'command' topic.
    """
    
    AVAILABLE_ACTIONS = [
        'sit', 'stand', 'forward', 'backward', 'turn_left', 'turn_right',
        'stretch', 'push_up', 'hand_shake', 'scratch', 'high_five',
        'shake_head', 'wag_tail', 'head_tilt', 'look_around', 'bark',
        'howling', 'startle', 'stop'
    ]
    
    def __init__(self):
        super().__init__('pidog_movement_node')
        
        #self.get_logger().info("Movement node starting (NO hardware initialization)")
        
        # Parameters
        self.declare_parameter('default_speed', 70)
        self.declare_parameter('default_step_count', 5)
        
        self.default_speed = self.get_parameter('default_speed').value
        self.default_step_count = self.get_parameter('default_step_count').value
        
        # Publisher for commands (to main node)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.command_pub = self.create_publisher(String, 'command', qos_profile)
        
        # Services for individual movement commands
        self.sit_srv = self.create_service(Trigger, 'sit', self.make_service_callback('sit'))
        self.stand_srv = self.create_service(Trigger, 'stand', self.make_service_callback('stand'))
        self.forward_srv = self.create_service(Trigger, 'forward', self.make_service_callback('forward'))
        self.backward_srv = self.create_service(Trigger, 'backward', self.make_service_callback('backward'))
        self.turn_left_srv = self.create_service(Trigger, 'turn_left', self.make_service_callback('turn_left'))
        self.turn_right_srv = self.create_service(Trigger, 'turn_right', self.make_service_callback('turn_right'))
        self.stop_srv = self.create_service(Trigger, 'stop', self.make_service_callback('stop'))
        
        self.get_logger().info("PiDog Movement Node Ready (forwarding commands to main node)")
        self.get_logger().info("Available actions: " + ", ".join(self.AVAILABLE_ACTIONS))
    
    def make_service_callback(self, command: str):
        """Create service callback that forwards command to main node."""
        def callback(request, response):
            self.get_logger().info(f"Service call: {command}")
            self.forward_command(command)
            response.success = True
            response.message = f"{command} command sent to main node"
            return response
        return callback
    
    def forward_command(self, command: str, step_count: int = None, speed: int = None):
        """Forward command to main node via topic."""
        if step_count is None:
            step_count = self.default_step_count
        if speed is None:
            speed = self.default_speed
        
        # Format command
        cmd_str = f"{command}:{step_count}:{speed}"
        
        msg = String()
        msg.data = cmd_str
        self.command_pub.publish(msg)
        self.get_logger().info(f"Forwarded command: {cmd_str}")
    
    def shutdown(self):
        self.get_logger().info("Movement node shutting down")


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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
