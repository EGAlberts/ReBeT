#!/bin/bash
source ~/.bashrc
source ~/rebet_ws/install/setup.bash

#ros2 run rebet arborist 
ros2 launch rebet arborist_launch.py experiment_name:=$1 tree_name:=$2 task_qa_window:=$3 power_qa_window:=$4
