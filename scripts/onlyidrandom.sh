#!/bin/bash
cd scripts

gnome-terminal --profile=closeonkill  -- ./rosanything.sh random_action_servers_launch.py
sleep 3
gnome-terminal --profile=closeonkill -- ./darknet.sh
sleep 1
gnome-terminal --profile=closeonkill -- ./gazebo_tb3.sh 
sleep 15
gnome-terminal --profile=closeonkill -- ./nav2_map.sh 
sleep 15
gnome-terminal --profile=closeonkill -- ./arborist.sh $1 onlyIDRandom.xml
sleep 5
gnome-terminal --profile=closeonkill -- ./sysrefl.sh 
sleep 3
gnome-terminal --profile=closeonkill -- ./tree_action.sh 
