#!/bin/bash

ros2 run rosserial_python serial_node--ros-args -p port:=/dev/ttyACM0 & 
ros2 run yuvaan_controller controller.py & 
python3 ~/app.py
