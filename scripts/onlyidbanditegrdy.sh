#!/bin/bash

gnome-terminal --profile=closeonkill -- ./scripts/rosanything.sh bandit_action_servers_launch.py
sleep 3
gnome-terminal --profile=closeonkill -- ./scripts/darknet.sh
sleep 1
gnome-terminal --profile=closeonkill -- ./scripts/gazebo_tb3.sh 
sleep 15
gnome-terminal --profile=closeonkill -- ./scripts/nav2_map.sh 
sleep 15
gnome-terminal --profile=closeonkill -- ./scripts/arborist.sh $1 onlyIDBanditgrdy.xml
sleep 5
gnome-terminal --profile=closeonkill -- ./scripts/sysrefl.sh 
sleep 3
gnome-terminal --profile=closeonkill -- ./scripts/tree_action.sh 
