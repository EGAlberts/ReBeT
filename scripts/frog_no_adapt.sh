#!/bin/bash

trap ctrl_c INT

function ctrl_c() {
        echo "Cleaning up.."
	./docker_kill_gazebo.sh	
	./killall.sh
        echo "Shutting down"
        exit 0
}


cd scripts




for EXPNUM in {1..5}
do
	gnome-terminal -- ./darknet.sh
	sleep 15
	gnome-terminal -- ./frontier_service.sh
	sleep 3
	gnome-terminal -- ./docker_gazebo_tb3.sh false 1
	sleep 15
	gnome-terminal -- ./nav2.sh 
	sleep 15
	gnome-terminal -- ./arborist.sh test frog_norebet.xml 0 0
	sleep 3
	gnome-terminal -- ./sys_refl.sh
	sleep 3
	gnome-terminal -- ./tree_action.sh 
	SECONDS=0 
	while [ ! -f ~/rebet_ws/scripts/mission.done -a $SECONDS -lt 700 ]
	do
	   echo "waiting for mission to be done" $EXP             
	sleep 10 #sustainability!
	done
	echo "mission done"
	rm ~/rebet_ws/scripts/mission.done
	./docker_kill_gazebo.sh	
	./killall.sh	
done
