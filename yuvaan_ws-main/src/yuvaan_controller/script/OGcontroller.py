#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from yuvaan_controller.msg import yuvaan

# Global variable for current mode index
current_mode_index = 0
modes = [1, 2, 3]

# mani_mode_index = 0
# mani_modes = [30,60,80]


def joy_callback(msg):
    global current_mode_index  # Use the global variable

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
    # ra_1 = ra_2 = ra_3 = ra_4 = ra_5 = ra_6 = 0
    DRILL = ACTUATOR = NPK = EXTRA = ba_5 = ba_6 = 0

    # Check if BACK button is pressed
    if BACK == 1:
        current_mode_index = (current_mode_index + 1) % len(modes)

    mode = modes[current_mode_index]

    if mode == 1:
        vel_linear_x = int(127 * (RT - LT) / 2)
        vel_angular_z = int(127 * L_Analog_X)

    elif mode == 2:
        vel_linear_x = int(127 * D_Y)
        vel_angular_z = int(127 * D_X)
        BASE = int(255 * L_Analog_X)
        SHOULDER = int(255 * L_Analog_Y)
        ELBOW = int(255 * R_Analog_Y)
        ROLL = int(-30 * R_Analog_X)
        PITCH = int(30 * (RT - LT) / 2)
        GRIPPER = int(127 * (RB - LB))

    elif mode == 3:
        vel_linear_x = int(255 * (RT - LT) / 2)
        vel_angular_z = int(255 * L_Analog_X)
        DRILL = int(255 * (RB - LB))
        ACTUATOR = int(255 * D_Y)
        NPK = int(255 * (Y - A))
        EXTRA = int(255 * R_Analog_X)

    motorspeed = yuvaan()
    motorspeed.mode = mode
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

    # Publish motor control command
    motor_command_pub.publish(motorspeed)

if __name__ == '__main__':
    rospy.init_node('motor_control_node')
    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
    motor_command_pub = rospy.Publisher('motor_command', yuvaan, queue_size=10)
    rospy.spin()
