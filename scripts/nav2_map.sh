#!/bin/bash
source ~/.bashrc
source ~/rebet_ws/install/setup.bash

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False map:=/home/$USER/rebet_ws/map_1695402756.yaml
