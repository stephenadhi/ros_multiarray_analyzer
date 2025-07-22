#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import (UInt32MultiArray, Float32MultiArray, Int32MultiArray)
import numpy as np

class TestArrayPublisher(Node):
    def __init__(self):
        super().__init__('test_array_publisher')
        
        # Create publishers for different types
        self.uint32_pub = self.create_publisher(
            UInt32MultiArray, 
            'test_uint32_array', 
            10
        )
        
        self.float32_pub = self.create_publisher(
            Float32MultiArray,
            'test_float32_array',
            10
        )
        
        self.int32_pub = self.create_publisher(
            Int32MultiArray,
            'test_int32_array',
            10
        )
        
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.counter = 0
        
    def timer_callback(self):
        # Create sample data
        uint32_msg = UInt32MultiArray()
        uint32_msg.data = [i + self.counter for i in range(300)]  # 300 incrementing values
        
        float32_msg = Float32MultiArray()
        float32_msg.data = [np.sin(i * 0.1 + self.counter * 0.1) for i in range(100)]  # Sine wave
        
        int32_msg = Int32MultiArray()
        int32_msg.data = [int(50 * np.sin(i * 0.1 + self.counter * 0.1)) for i in range(150)]  # Scaled sine wave
        
        # Publish messages
        self.uint32_pub.publish(uint32_msg)
        self.float32_pub.publish(float32_msg)
        self.int32_pub.publish(int32_msg)
        
        self.counter += 1

def main():
    # Get domain ID from environment variable
    domain_id = int(os.getenv('ROS_DOMAIN_ID', '0'))
    print(f"Publishing on ROS_DOMAIN_ID: {domain_id}")
    
    rclpy.init()
    node = TestArrayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 