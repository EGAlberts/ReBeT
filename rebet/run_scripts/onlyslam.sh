#!/bin/bash

cd scripts

gnome-terminal -- ./nav2.sh 
sleep 15
gnome-terminal -- ./arborist.sh $1 onlySLAM.xml $2 $3
sleep 5
gnome-terminal -- ./adap_sys.sh 10
sleep 3
gnome-terminal -- ./tree_action.sh 
