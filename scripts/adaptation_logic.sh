#!/bin/bash
source ~/.bashrc
source ~/rebet_ws/install/setup.bash

pwd


if [ "$2" != "none" ];
then
	if [ "$3" != "none" ];
	then
    	echo "bandit"
    		ros2 launch rebet adaptation_logic_launch.py adaptation_logic:=$1 bandit_name:=$2 hyperparameters:=$3
	else
		ros2 launch rebet adaptation_logic_launch.py adaptation_logic:=$1 bandit_name:=$2
	fi
else
    echo "not bandit"
    ros2 launch rebet adaptation_logic_launch.py adaptation_logic:=$1
fi


