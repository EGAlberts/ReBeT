#!/bin/bash
source ~/.bashrc
source ~/rebet_ws/install/setup.bash
source /usr/share/gazebo/setup.bash

export GAZEBO_MODEL_PATH=/home/kasm-user/rebet_ws/install/rebet_sim/share/rebet_sim/models/
export TURTLEBOT3_MODEL=waffle

ros2 launch rebet_sim spawn_tb3.launch.py gui:=$1 myseed:=$2
