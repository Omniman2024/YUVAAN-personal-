#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from yuvaan_controller.msg import yuvaan


yaw_mode_index = 0
yaw_modes = [30,60,80]

roll_mode_index = 0
roll_modes = [30,60,80]


def joy_callback(msg):
    global yaw_mode_index
    global roll_mode_index

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

    # vel_linear_x = 0
    # vel_angular_z = 0
    BASE = SHOULDER = ELBOW = ROLL = PITCH = GRIPPER = 0
    # # ra_1 = ra_2 = ra_3 = ra_4 = ra_5 = ra_6 = 0
    # DRILL = ACTUATOR = NPK = EXTRA = ba_5 = ba_6 = 0


    # Check if A button is pressed
    if A == 1:
        yaw_mode_index = (yaw_mode_index + 1) % len(yaw_modes)

    yaw_mode = yaw_modes[yaw_mode_index]

    # Check if B button is pressed
    if B == 1:
        roll_mode_index = (roll_mode_index + 1) % len(roll_modes)

    roll_mode = roll_modes[roll_mode_index]


    vel_linear_x = int(97 * D_Y)
    vel_angular_z = int(97 * D_X)
    BASE = int(255 * L_Analog_X)
    SHOULDER = int(255 * L_Analog_Y)
    ELBOW = int(255 * R_Analog_Y)
    ROLL = int(-roll_mode * R_Analog_X)
    PITCH = int(yaw_mode * (RT - LT) / 2)
    GRIPPER = int(127 * (RB - LB))

    motorspeed = yuvaan()
    motorspeed.roll_mode = roll_mode
    motorspeed.yaw_mode = yaw_mode
    motorspeed.ra_1 = BASE
    motorspeed.ra_2 = SHOULDER
    motorspeed.ra_3 = ELBOW
    motorspeed.ra_4 = ROLL
    motorspeed.ra_5 = PITCH
    motorspeed.ra_6 = GRIPPER

    # Publish motor control command
    motor_command_pub.publish(motorspeed)

if __name__ == '__main__':
    rospy.init_node('motor_control_node')
    joy_sub = rospy.Subscriber('joy_b', Joy, joy_callback)
    motor_command_pub = rospy.Publisher('motor_command', yuvaan, queue_size=10)
    rospy.spin()
