#!/usr/bin/env python3

import cv2
import numpy as np
import math
import pyzed.sl as sl
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge

# Constants
MATCH_THRESHOLD = 0.8
DEPTH_THRESHOLD = 2.0

# Cone detection parameters
MIN_NBR_PIXELS = 50
MAX_NBR_PIXELS = 10000
MIN_CLUSTER_RATIO = 0.4
MAX_CLUSTER_RATIO = 2.0
LIM_CLUSTER_RATIO = 1.0
MIN_BOX_RATIO = 0.3
MAX_BOX_RATIO = 1.5
GROUP_PERCENT = 0.3
CENTER_PERCENT = 0.3
GAUSSIAN_SIZE = 2
BOTTOM_RATIO = 0.7

# Color ranges for cone detection
YELLOW_MIN = np.array([10, 80, 200])
YELLOW_MAX = np.array([25, 255, 255])
BLACK_MIN = np.array([0, 0, 0])
BLACK_MAX = np.array([50, 255, 180])

class CombinedDetectionNode:
    def __init__(self):
        rospy.init_node('combined_detection_node')
        
        # Load arrow templates
        right_arrow_path = rospy.get_param('~right_arrow_path', "/home/jetson/Downloads/right.png")
        left_arrow_path = rospy.get_param('~left_arrow_path', "/home/jetson/Downloads/left.png")
        self.right_arrow = cv2.imread(right_arrow_path, cv2.IMREAD_GRAYSCALE)
        self.left_arrow = cv2.imread(left_arrow_path, cv2.IMREAD_GRAYSCALE)
        
        # Publishers
        self.arrow_direction_pub = rospy.Publisher('arrow_direction', String, queue_size=10)
        self.arrow_depth_pub = rospy.Publisher('arrow_depth', Float32, queue_size=10)
        self.yellow_cone_pub = rospy.Publisher('yellow_cone_detected', String, queue_size=10)
        self.cone_depth_pub = rospy.Publisher('cone_depth', Float32, queue_size=10)
        
        self.bridge = CvBridge()
        rospy.loginfo("Combined detection node initialized")

    def find_bounding_boxes(self, img, min_main, max_main, min_center, max_center):
        """Cone detection function from the original code"""
        (init_height, init_width, _) = img.shape
        x1 = int((1-BOTTOM_RATIO) * init_height)
        x2 = init_height
        img_crop = img[x1:x2, :, :]
        
        hsv = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, min_main, max_main)
        
        outputs = cv2.connectedComponentsWithStats(mask, 8)
        nbr_clusters = outputs[0]
        labels = outputs[1]
        stats = outputs[2]
        centroids = outputs[3]
        
        bounding_boxes = []
        for k in range(nbr_clusters):
            area = stats[k][cv2.CC_STAT_AREA]
            w = stats[k][cv2.CC_STAT_WIDTH]
            h = stats[k][cv2.CC_STAT_HEIGHT]
            
            if MIN_NBR_PIXELS <= area <= MAX_NBR_PIXELS:
                ratio = w/float(h)
                if MIN_CLUSTER_RATIO <= ratio <= MAX_CLUSTER_RATIO:
                    x = int(stats[k][cv2.CC_STAT_LEFT])
                    y = int(stats[k][cv2.CC_STAT_TOP])
                    bounding_boxes.append((y+x1, x, h, w))
        
        return bounding_boxes

    def detect_arrow(self, frame):
        """Arrow detection from the original code"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edges = cv2.Canny(blurred, 50, 150)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
            hull_indices = cv2.convexHull(approx, returnPoints=False)
            
            if (len(hull_indices) > 4 and len(hull_indices) < 6 and 
                len(hull_indices) + 2 == len(approx) and len(approx) > 6):
                
                tip = self.identify_arrow_tip(approx[:, 0], hull_indices.flatten())
                if tip[0] != -1 and tip[1] != -1:
                    direction = self.determine_direction(approx, tip)
                    if direction != "None":
                        return tip, direction
        return None, None

    def identify_arrow_tip(self, points, hull_indices):
        remaining_indices = [i for i in range(len(points)) if i not in hull_indices]
        for i in range(2):
            j = (remaining_indices[i] + 2) % len(points)
            if (points[j] == points[(remaining_indices[i - 1] - 2 + len(points)) % len(points)]).all():
                return points[j]
        return (-1, -1)

    def determine_direction(self, approx, tip):
        left_points = sum(1 for pt in approx if pt[0][0] < tip[0])
        right_points = sum(1 for pt in approx if pt[0][0] > tip[0])
        
        if left_points > right_points and left_points > 4:
            return "Left"
        if right_points > left_points and right_points > 4:
            return "Right"
        return "None"

    def get_depth(self, zed, runtime_params, point):
        depth_map = sl.Mat()
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)
            z_depth = depth_map.get_value(int(point[0]), int(point[1]))[1]
            return z_depth
        return None

    def process_detection(self):
        # Step 1: Open OpenCV Camera
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            rospy.logerr("Error opening video capture")
            return

        ret, frame = cap.read()
        if not ret or frame is None:
            rospy.logerr("Error capturing frame")
            cap.release()
            return

        # Step 2: Process Frame for both arrow and cone detection
        # Arrow detection
        arrow_point, direction = self.detect_arrow(frame)
        
        # Cone detection
        yellow_boxes = self.find_bounding_boxes(frame, YELLOW_MIN, YELLOW_MAX, BLACK_MIN, BLACK_MAX)

        # Close OpenCV Camera
        cap.release()

        # Step 3: If either arrow or cone is detected, open ZED camera for depth
        if direction is not None or yellow_boxes:
            zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.coordinate_units = sl.UNIT.METER
            init_params.depth_mode = sl.DEPTH_MODE.ULTRA
            
            if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
                rospy.logerr("Error opening ZED camera")
                return

            runtime_params = sl.RuntimeParameters()

            # Get depth for arrow if detected
            if direction is not None:
                z_depth = self.get_depth(zed, runtime_params, arrow_point)
                if z_depth is not None:
                    self.arrow_direction_pub.publish(String(direction))
                    self.arrow_depth_pub.publish(Float32(z_depth))
                    rospy.loginfo(f"Arrow detected: {direction} at {z_depth:.2f}m")

            # Get depth for cones if detected
            for box in yellow_boxes:
                center_point = (int(box[1] + box[3]/2), int(box[0] + box[2]/2))
                depth = self.get_depth(zed, runtime_params, center_point)
                if depth is not None and depth < DEPTH_THRESHOLD:
                    self.yellow_cone_pub.publish(String("Yellow cone detected"))
                    self.cone_depth_pub.publish(Float32(depth))
                    rospy.loginfo(f"Yellow cone detected at {depth:.2f}m")

            # Close ZED Camera
            zed.close()

        cv2.destroyAllWindows()

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.process_detection()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = CombinedDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass