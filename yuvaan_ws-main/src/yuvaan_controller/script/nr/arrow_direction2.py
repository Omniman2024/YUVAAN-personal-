#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String

class DirectionFilterNode(Node):
    def __init__(self):
        super().__init__('direction_filter_node')
        
        # Store the latest distance reading (in centimeters)
        self.current_distance = float('inf')
        
        # Create publisher for filtered direction
        self.filtered_pub = self.create_publisher(String, 'filtered_direction', 10)
        
        # Subscribe to distance and arrow direction topics
        self.distance_sub = self.create_subscription(
            Float32,
            'ultrasonic_distance',
            self.distance_callback,
            10)
        
        self.direction_sub = self.create_subscription(
            String,
            'arrow_direction',
            self.direction_callback,
            10)
            
        self.get_logger().info("Direction filter node initialized")
    
    def distance_callback(self, msg):
        """Update the stored distance value (incoming in cm)"""
        self.current_distance = msg.data
    
    def direction_callback(self, msg):
        """Process incoming direction and publish if conditions are met"""
        direction = msg.data
        
        # Convert distance to meters for comparison (200 cm = 2 m)
        distance_in_meters = self.current_distance / 100.0
        
        # Check distance condition
        if distance_in_meters <= 2.0:
            filtered_msg = String()
            filtered_msg.data = direction
            self.filtered_pub.publish(filtered_msg)
            self.get_logger().info(f"Direction: {direction}, Distance: {distance_in_meters:.2f}m - Published")
        else:
            self.get_logger().info(f"Direction: {direction}, Distance: {distance_in_meters:.2f}m - Too far, not published")

def main(args=None):
    rclpy.init(args=args)
    node = DirectionFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
