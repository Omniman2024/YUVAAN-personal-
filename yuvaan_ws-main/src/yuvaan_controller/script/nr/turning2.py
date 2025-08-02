#!/usr/bin/env python3

import rospy
import pyrealsense2 as rs
import numpy as np
import time
import math
from std_msgs.msg import String
from yuvaan_controller.msg import yuvaan

def complementary_filter_update(ax, ay, az, gx, gy, gz, dt,
                                roll, pitch, yaw, alpha=0.98):
    roll_gyro  = roll  + gx * dt
    pitch_gyro = pitch + gy * dt
    yaw_gyro   = yaw   + gz * dt

    accel_norm = math.sqrt(ax*ax + ay*ay + az*az)
    if accel_norm > 1e-6:
        axn = ax / accel_norm
        ayn = ay / accel_norm
        azn = az / accel_norm

        pitch_acc = math.atan2(-axn, math.sqrt(ayn*ayn + azn*azn))
        roll_acc  = math.atan2(ayn, azn)

        roll  = alpha * roll_gyro  + (1 - alpha) * roll_acc
        pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
        yaw   = yaw_gyro
    else:
        roll  = roll_gyro
        pitch = pitch_gyro
        yaw   = yaw_gyro

    return roll, pitch, yaw

class ArrowTurnController:
    def _init_(self):
        rospy.init_node('arrow_turn_controller')
        
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
        self.first_turn_detected = False  # New flag to track first turn

        # ROS Subscribers and Publishers
        rospy.Subscriber('filtered_direction', String, self.arrow_callback)
        self.turn_pub = rospy.Publisher('motor_command', yuvaan, queue_size=10)

        # Initialize motor command message with forward motion
        self.motorspeed = yuvaan()
        self.motorspeed.mode = 1
        self.motorspeed.vel_linear_x = 50  # Start moving forward immediately
        self.motorspeed.vel_angular_z = 0
        self.motorspeed.ra_1 = 0
        self.motorspeed.ra_2 = 0
        self.motorspeed.ra_3 = 0
        self.motorspeed.ra_4 = 0
        self.motorspeed.ra_5 = 0
        self.motorspeed.ra_6 = 0
        self.motorspeed.ba_1 = 0
        self.motorspeed.ba_2 = 0
        self.motorspeed.ba_3 = 0
        self.motorspeed.ba_4 = 0
        self.motorspeed.ba_5 = 0
        self.motorspeed.ba_6 = 0
        
        # Start with forward motion
        self.turn_pub.publish(self.motorspeed)
        
        rospy.loginfo("Arrow Turn Controller Initialized - Moving forward")

    def arrow_callback(self, msg):
        if not self.first_turn_detected:
            self.first_turn_detected = True
            rospy.loginfo("First turn detected - Starting turn sequence")
            
        if self.is_turning:
            rospy.loginfo(f"Ignoring {msg.data} arrow - currently turning")
            return
        
        self.pending_turn = msg.data
        self.initiate_turn()

    def initiate_turn(self):
        if not self.pending_turn or self.is_turning:
            return
        
        # Stop linear motion and start turning
        self.motorspeed.vel_linear_x = 0
        self.motorspeed.vel_angular_z = 50 if self.pending_turn == "Left" else -50

        # Publish motor control command
        self.turn_pub.publish(self.motorspeed)
        
        # Mark that we're now turning
        self.is_turning = True
        rospy.loginfo(f"Initiating {self.pending_turn} turn")
        
        # Clear pending turn
        self.pending_turn = None

    def check_turn_completion(self):
        frames = self.pipeline.poll_for_frames()
        if not frames:
            return False

        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame  = frames.first_or_default(rs.stream.gyro)
        if not accel_frame or not gyro_frame:
            return False

        # Extract motion data
        accel_data = accel_frame.as_motion_frame().get_motion_data()
        gyro_data  = gyro_frame.as_motion_frame().get_motion_data()

        ax, ay, az = accel_data.x, accel_data.y, accel_data.z
        gx, gy, gz = gyro_data.x, gyro_data.y, gyro_data.z

        # Compute delta time
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Update roll, pitch, yaw
        self.roll, self.pitch, self.yaw = complementary_filter_update(
            ax, ay, az, gx, gy, gz, dt, 
            self.roll, self.pitch, self.yaw, alpha=0.98
        )

        # Convert yaw to degrees
        yaw_degs = math.degrees(self.yaw)

        # Compare current yaw with reference to detect 90° turns
        delta = yaw_degs - self.reference_yaw_degs

        # Normalize the difference to (-180, 180]
        if delta > 180:
            delta -= 360
        elif delta <= -180:
            delta += 360

        # Check if turn is complete
        if abs(delta) >= self.TURN_THRESHOLD:
            # Reset reference yaw
            self.reference_yaw_degs += 90 if delta > 0 else -90

            # Reset to forward motion
            self.motorspeed.vel_linear_x = 50
            self.motorspeed.vel_angular_z = 0
            self.turn_pub.publish(self.motorspeed)

            return True

        return False

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.is_turning and self.check_turn_completion():
                self.is_turning = False
                rospy.loginfo("Turn completed. Ready for next arrow.")
            rate.sleep()

        # Cleanup
        self.pipeline.stop()

if _name_ == '_main_':
    try:
        controller = ArrowTurnController()
        controller.run()
    except rospy.ROSInterruptException:
        pass