#!/bin/bash

cd yuvaan_ws/src/yuvaan_controller/script
ros2 run rosserial_python serial_node--ros-args -p port:=/dev/ttyACM0 & 
ros2 run yuvaan_controller controller.py &
ros2 launch rosbridge_server rosbridge_websocket.launch & 
python3 ~/app.py &
python3 test2.py
