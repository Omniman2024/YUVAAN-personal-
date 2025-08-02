#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import time
import math
from std_msgs.msg import String
from yuvaan_controller.msg import Yuvaan # Message name is now PascalCase

# This helper function is independent of ROS and does not need to change
def complementary_filter_update(ax, ay, az, gx, gy, gz, dt,
                                roll, pitch, yaw, alpha=0.98):
    # ... (function content is unchanged)
    return roll, pitch, yaw

class ArrowTurnController(Node):
    def __init__(self):
        super().__init__('arrow_turn_controller')
        
        # IMU and turn detection setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)
        self.pipeline.start(config)

        # State tracking
        self.is_turning = False
        self.pending_turn = None
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.prev_time = time.time()
        self.reference_yaw_degs = 0.0
        self.TURN_THRESHOLD = 90.0

        # ROS Subscribers and Publishers
        self.subscription = self.create_subscription(
            String,
            'filtered_direction',
            self.arrow_callback,
            10)
        self.turn_pub = self.create_publisher(Yuvaan, 'motor_command', 10)

        # Initialize motor command message with forward motion
        self.motorspeed = Yuvaan() # Message type is now PascalCase
        self.motorspeed.mode = 1
        self.motorspeed.vel_linear_x = 50
        # ... (rest of the message fields)

        # Start with forward motion
        self.turn_pub.publish(self.motorspeed)
        
        self.get_logger().info("Arrow Turn Controller Initialized")
        
        # Create a timer for the main loop
        self.timer = self.create_timer(0.1, self.run_callback) # 10 Hz

    def run_callback(self):
        if self.is_turning and self.check_turn_completion():
            self.is_turning = False
            self.get_logger().info("Turn completed. Ready for next arrow.")
            
    # ... (arrow_callback, initiate_turn, check_turn_completion methods are the same) ...
    # ... Just remember to change rospy.loginfo to self.get_logger().info() ...

    def cleanup(self):
        self.pipeline.stop()


def main(args=None):
    rclpy.init(args=args)
    controller = ArrowTurnController()
    rclpy.spin(controller)
    controller.cleanup()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
