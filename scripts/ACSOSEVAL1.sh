#!/bin/bash

trap ctrl_c INT

function ctrl_c() {
        echo "Cleaning up.."
	./killall.sh
        echo "Shutting down"
        exit 0
}


cd scripts


for EXPNUM in {1..1}
do
	xfce4-terminal -e ./frontier_service.sh
	sleep 3
	xfce4-terminal -e "./start_gazebo.sh true 1"
	sleep 15
	xfce4-terminal -e ./nav2.sh 
	sleep 15
	xfce4-terminal -e "./arborist.sh frog_task_based.xml 8"
	sleep 3
	xfce4-terminal -e ./sys_refl.sh
	sleep 3
	xfce4-terminal -e ./report.sh
	sleep 3
	xfce4-terminal -e ./tree_action.sh 
	SECONDS=0 
	while [ ! -f ~/rebet_ws/scripts/mission.done -a $SECONDS -lt 700 ]
	do
	   echo "waiting for mission to be done" $EXP             
	sleep 10 #sustainability!
	done
	echo "mission done"
	rm ~/rebet_ws/scripts/mission.done
	./killall.sh	
done


