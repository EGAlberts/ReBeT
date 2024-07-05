#!/bin/bash
source ~/.bashrc
source ~/rebet_ws/install/setup.bash

#ros2 run rebet arborist 
ros2 launch rebet arborist_launch.py tree_name:=$1 power_qa_window:=$2

