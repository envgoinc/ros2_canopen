#!/usr/bin/env python3
"""
Test script for the raw CAN bridge
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import time


class BridgeTester(Node):
    def __init__(self):
        super().__init__('bridge_tester')
        
        # Publisher to send distances
        self.distance_publisher = self.create_publisher(
            UInt16MultiArray,
            'distances_to_send',
            10
        )
        
        # Subscriber to receive distances
        self.distance_subscriber = self.create_subscription(
            UInt16MultiArray,
            'distances_received',
            self.distances_received_callback,
            10
        )
        
        # Timer to send test data
        self.timer = self.create_timer(2.0, self.send_test_distances)
        
        self.get_logger().info('Bridge Tester started')
    
    def send_test_distances(self):
        """Send test distance values"""
        msg = UInt16MultiArray()
        msg.data = [65535, 32000, 10000, 5000]  # Your test values
        
        self.distance_publisher.publish(msg)
        self.get_logger().info(f'Sent distances: {list(msg.data)}')
    
    def distances_received_callback(self, msg):
        """Handle received distances"""
        self.get_logger().info(f'Received distances: {list(msg.data)}')


def main(args=None):
    rclpy.init(args=args)
    node = BridgeTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

