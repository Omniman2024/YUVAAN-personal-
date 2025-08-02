#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from yuvaan_controller.msg import Yuvaan  # Note the PascalCase for the message type

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Class attributes replace global variables
        self.current_mode_index = 0
        self.modes = [1, 2, 3, 4]
        self.yaw_mode_index = 0
        self.yaw_modes = [30, 60, 80]
        self.roll_mode_index = 0
        self.roll_modes = [30, 60, 80]

        # Create the publisher and subscriber
        self.motor_command_pub = self.create_publisher(Yuvaan, 'motor_command', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        # The logic from your original callback goes here.
        # Global variables are now instance attributes (e.g., self.current_mode_index)
        
        LT = -msg.axes[2]
        RT = -msg.axes[5]
        LB = msg.buttons[4]
        RB = msg.buttons[5]
        L_Analog_Y = msg.axes[1]
        L_Analog_X = -msg.axes[0]
        R_Analog_Y = msg.axes[4]
        R_Analog_X = msg.axes[3]
        D_Y = msg.axes[7]
        D_X = msg.axes[6]
        Y = msg.buttons[3]
        X = msg.buttons[2]
        B = msg.buttons[1]
        A = msg.buttons[0]
        START = msg.buttons[7]
        BACK = msg.buttons[6]
        LOGITECH = msg.buttons[8]

        vel_linear_x = 0
        vel_angular_z = 0
        BASE = SHOULDER = ELBOW = ROLL = PITCH = GRIPPER = 0
        DRILL = ACTUATOR = NPK = EXTRA = ba_5 = ba_6 = 0

        # Check if BACK button is pressed
        if BACK == 1:
            self.current_mode_index = (self.current_mode_index + 1) % len(self.modes)
        mode = self.modes[self.current_mode_index]

        # Check if A button is pressed
        if A == 1:
            self.yaw_mode_index = (self.yaw_mode_index + 1) % len(self.yaw_modes)
        yaw_mode = self.yaw_modes[self.yaw_mode_index]

        # Check if B button is pressed
        if B == 1:
            self.roll_mode_index = (self.roll_mode_index + 1) % len(self.roll_modes)
        roll_mode = self.roll_modes[self.roll_mode_index]

        if mode == 1:
            vel_linear_x = int(97 * (RT - LT) / 2)
            vel_angular_z = int(97 * L_Analog_X)
        elif mode == 2:
            vel_linear_x = int(127 * (RT - LT) / 2)
            vel_angular_z = int(127 * L_Analog_X)
        elif mode == 3:
            vel_linear_x = int(97 * D_Y)
            vel_angular_z = int(97 * D_X)
            BASE = int(255 * L_Analog_X)
            SHOULDER = int(255 * L_Analog_Y)
            ELBOW = int(255 * R_Analog_Y)
            ROLL = int(-roll_mode * R_Analog_X)
            PITCH = int(yaw_mode * (RT - LT) / 2)
            GRIPPER = int(127 * (RB - LB))
        elif mode == 4:
            vel_linear_x = int(255 * (RT - LT) / 2)
            vel_angular_z = int(255 * L_Analog_X)
        
        # Note the PascalCase for the message type constructor
        motorspeed = Yuvaan()
        motorspeed.mode = mode
        motorspeed.roll_mode = roll_mode
        motorspeed.yaw_mode = yaw_mode
        motorspeed.vel_linear_x = vel_linear_x
        motorspeed.vel_angular_z = vel_angular_z
        motorspeed.ra_1 = BASE
        motorspeed.ra_2 = SHOULDER
        motorspeed.ra_3 = ELBOW
        motorspeed.ra_4 = ROLL
        motorspeed.ra_5 = PITCH
        motorspeed.ra_6 = GRIPPER
        motorspeed.ba_1 = DRILL
        motorspeed.ba_2 = ACTUATOR
        motorspeed.ba_3 = NPK
        motorspeed.ba_4 = EXTRA
        motorspeed.ba_5 = ba_5
        motorspeed.ba_6 = ba_6

        # Publish motor control command using the instance's publisher
        self.motor_command_pub.publish(motorspeed)

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    
    # Destroy the node explicitly
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
