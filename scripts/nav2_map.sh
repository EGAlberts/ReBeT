#!/bin/bash
source ~/.bashrc
source ~/rebet_ws/install/setup.bash

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=/home/$USER/rebet_ws/map_1713963103.yaml params_file:=/home/$USER/rebet_ws/frog_nav2_params.yaml
