#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import pyzed.sl as sl
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge

MATCH_THRESHOLD = 0.8
DEPTH_THRESHOLD = 2.0

class ArrowDetectionNode(Node):
    def __init__(self):
        super().__init__('arrow_detection_node')
        self.get_logger().info("System initialization...")

        # Declare and get parameters
        self.declare_parameter('right_arrow_path', '/home/jetson/Downloads/right.png')
        self.declare_parameter('left_arrow_path', '/home/jetson/Downloads/left.png')
        right_arrow_path = self.get_parameter('right_arrow_path').get_parameter_value().string_value
        left_arrow_path = self.get_parameter('left_arrow_path').get_parameter_value().string_value

        # Load templates
        self.right_arrow = cv2.imread(right_arrow_path, cv2.IMREAD_GRAYSCALE)
        self.left_arrow = cv2.imread(left_arrow_path, cv2.IMREAD_GRAYSCALE)
        self.get_logger().info(f"Template Right_Arrow loaded.")
        self.get_logger().info(f"Template Left_Arrow loaded.")

        # Publishers
        self.direction_pub = self.create_publisher(String, 'arrow_direction', 10)
        self.depth_pub = self.create_publisher(Float32, 'arrow_depth', 10)

        # OpenCV Bridge
        self.bridge = CvBridge()
        
        # Replace the run() method and while loop with a timer
        timer_period = 1.0  # seconds (equivalent to Rate(1))
        self.timer = self.create_timer(timer_period, self.process_detection)

    # All your existing helper methods (edge_detection, detect_contours, etc.) go here
    # ... (omitted for brevity, they do not need changes) ...

    def process_detection(self):
        # NOTE: Opening and closing cameras in a loop is inefficient.
        # A better design would be to open them once in __init__ and grab frames in the callback.
        # This code is a direct migration of the original logic.
        
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Error opening video capture")
            return
        # ... (rest of the process_detection logic is the same) ...
        cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ArrowDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
