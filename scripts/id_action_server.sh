#!/bin/bash
source ~/.bashrc
source ~/rebet_ws/install/setup.bash

ros2 run rebet ID_action_server.py --ros-args -p pic_rate:=$1 -p det_threshold:=$2
