#!/bin/bash
source ~/.bashrc
source ~/rebet_ws/install/setup.bash
source /usr/share/gazebo/setup.sh

export TURTLEBOT3_MODEL=waffle

ros2 launch rebet spawn_tb3.launch.py gui:='true'
