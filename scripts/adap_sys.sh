#!/bin/bash
source ~/.bashrc
source ~/rebet_ws/install/setup.bash

ros2 launch rebet adaptation_system_launch.py adaptation_period:=$1
