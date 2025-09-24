#!/usr/bin/env python3
"""
Raw CAN to ROS Bridge
Simple bridge for packed multi-value CAN messages
"""

import rclpy
from rclpy.node import Node
import can
import struct
from std_msgs.msg import UInt16MultiArray
import threading
import time


class DistanceMessage:
    """Custom message for 4 distance values"""
    def __init__(self, distances=None):
        self.distances = distances or [0, 0, 0, 0]
    
    def pack_can_frame(self, can_id=0x202):
        """Pack 4 distances into 8-byte CAN frame"""
        # Pack as 4 x 16-bit unsigned integers, big-endian to match DBC
        data = struct.pack('>HHHH', *self.distances)
        return can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    
    @classmethod
    def from_can_frame(cls, can_msg):
        """Unpack CAN frame into 4 distances"""
        if len(can_msg.data) >= 8:
            distances = list(struct.unpack('>HHHH', can_msg.data[:8]))
            return cls(distances)
        return None


class RawCanBridge(Node):
    def __init__(self):
        super().__init__('raw_can_bridge')
        
        # Parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_id', 0x202)
        self.declare_parameter('bitrate', 500000)
        
        can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self.can_id = self.get_parameter('can_id').get_parameter_value().integer_value
        
        # Initialize CAN interface
        try:
            self.can_bus = can.interface.Bus(
                channel=can_interface,
                bustype='socketcan',
                receive_own_messages=False
            )
            self.get_logger().info(f'Connected to CAN interface: {can_interface}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CAN: {e}')
            return
        
        # ROS Publishers and Subscribers
        self.distance_publisher = self.create_publisher(
            UInt16MultiArray,
            'distances_received',
            10
        )
        
        self.distance_subscriber = self.create_subscription(
            UInt16MultiArray,
            'distances_to_send',
            self.send_distances_callback,
            10
        )
        
        # Start CAN receiver thread
        self.running = True
        self.can_thread = threading.Thread(target=self.can_receiver_thread)
        self.can_thread.daemon = True
        self.can_thread.start()
        
        self.get_logger().info('Raw CAN Bridge started')
    
    def send_distances_callback(self, msg):
        """Send distances as CAN message"""
        if len(msg.data) != 4:
            self.get_logger().error(f'Expected 4 distances, got {len(msg.data)}')
            return
        
        try:
            # Create distance message and pack into CAN frame
            dist_msg = DistanceMessage(list(msg.data))
            can_frame = dist_msg.pack_can_frame(self.can_id)
            
            # Send CAN frame
            self.can_bus.send(can_frame)
            
            self.get_logger().info(
                f'Sent CAN frame ID:{hex(self.can_id)} Data:{can_frame.data.hex().upper()}'
            )
            self.get_logger().info(f'Distances: {list(msg.data)}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')
    
    def can_receiver_thread(self):
        """Thread to receive CAN messages"""
        while self.running:
            try:
                # Receive CAN message with timeout
                can_msg = self.can_bus.recv(timeout=1.0)
                
                if can_msg and can_msg.arbitration_id == self.can_id:
                    # Unpack distance message
                    dist_msg = DistanceMessage.from_can_frame(can_msg)
                    
                    if dist_msg:
                        # Publish to ROS
                        ros_msg = UInt16MultiArray()
                        ros_msg.data = dist_msg.distances
                        self.distance_publisher.publish(ros_msg)
                        
                        self.get_logger().info(
                            f'Received CAN ID:{hex(can_msg.arbitration_id)} '
                            f'Distances:{dist_msg.distances}'
                        )
                
            except Exception as e:
                if self.running:  # Only log if we're supposed to be running
                    self.get_logger().warn(f'CAN receive error: {e}')
                time.sleep(0.1)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if hasattr(self, 'can_thread'):
            self.can_thread.join(timeout=2.0)
        if hasattr(self, 'can_bus'):
            self.can_bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RawCanBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
