#!/usr/bin/env python3
"""
ROS 2 Dual Touch Sensor Node - Listens to touch events from main node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy


class PiDogDualTouchNode(Node):
    def __init__(self):
        super().__init__('pidog_dual_touch_node')
        
        self.get_logger().info("Touch node starting - listening for touch events")
        # Publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        # Subscribe to touch events from main node
        self.touch_sub = self.create_subscription(
            String,
            'touch',
            self.touch_callback,
            qos_profile
        )
        
        # Re-publisher for processed touch events
        self.touch_pub = self.create_publisher(String, 'touch_event', 10)
        
        self.get_logger().info("Touch node ready - waiting for touch data")
    
    def touch_callback(self, msg: String):
        """Process and republish touch events"""
        self.get_logger().info(f"Touch received: {msg.data}")
        # Forward the event
        self.touch_pub.publish(msg)
    
    def shutdown(self):
        self.get_logger().info("Touch node shutting down")


def main(args=None):
    rclpy.init(args=args)
    node = PiDogDualTouchNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
