#!/bin/bash
cd scripts

gnome-terminal -- ./rosanything.sh bandit_action_servers_launch.py
sleep 3
gnome-terminal - -- ./darknet.sh
sleep 1
gnome-terminal - -- ./gazebo_tb3.sh 
sleep 15
gnome-terminal - -- ./nav2_map.sh 
sleep 15
gnome-terminal - -- ./arborist.sh $1 onlyIDBanditUCB.xml
sleep 5
gnome-terminal - -- ./sysrefl.sh 
sleep 3
gnome-terminal - -- ./tree_action.sh 
