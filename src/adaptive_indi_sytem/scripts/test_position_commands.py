#!/usr/bin/env python3
"""
Test script for Adaptive INDI System
Sends various position setpoints to test the controller
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import time

class PositionCommandPublisher(Node):
    def __init__(self):
        super().__init__('position_command_publisher')
        self.publisher = self.create_publisher(
            PointStamped,
            '/control/position_setpoint',
            10
        )
        
    def send_position(self, x, y, z):
        """Send a position setpoint"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent position: ({x:.2f}, {y:.2f}, {z:.2f})')

def main():
    rclpy.init()
    node = PositionCommandPublisher()
    
    print("\n" + "="*60)
    print("Adaptive INDI System - Position Command Test")
    print("="*60)
    print("\nMake sure the adaptive_indi_system_node is running!")
    print("Press Ctrl+C to stop\n")
    
    try:
        # Takeoff and hover
        print("[1/5] Takeoff to 2m altitude...")
        node.send_position(0.0, 0.0, -2.0)
        time.sleep(5)
        
        # Move forward
        print("[2/5] Move forward (X+)...")
        node.send_position(2.0, 0.0, -2.0)
        time.sleep(5)
        
        # Move right
        print("[3/5] Move right (Y+)...")
        node.send_position(2.0, 2.0, -2.0)
        time.sleep(5)
        
        # Return to origin
        print("[4/5] Return to origin...")
        node.send_position(0.0, 0.0, -2.0)
        time.sleep(5)
        
        # Descend
        print("[5/5] Descend to 1m...")
        node.send_position(0.0, 0.0, -1.0)
        time.sleep(5)
        
        print("\nTest sequence complete!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
