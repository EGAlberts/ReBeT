#!/bin/bash

cd scripts

gnome-terminal -- ./adaptation_logic.sh $4 $5 $6 
sleep 3
gnome-terminal -- ./task_launch.sh 'ID' $8 $9
sleep 3
gnome-terminal -- ./darknet.sh
sleep 1
gnome-terminal -- ./gazebo_tb3.sh 
sleep 15
gnome-terminal -- ./nav2_map.sh 
sleep 15
gnome-terminal -- ./arborist.sh $1 onlyID.xml $2 $3
sleep 5
gnome-terminal -- ./adap_sys.sh $7
sleep 3
gnome-terminal -- ./tree_action.sh 
