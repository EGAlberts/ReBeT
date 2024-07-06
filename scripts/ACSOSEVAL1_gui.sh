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
	mate-terminal -- ./frontier_service.sh
	sleep 3
	mate-terminal -- ./start_gazebo.sh true 1
	sleep 15
	mate-terminal -- ./nav2.sh 
	sleep 15
	mate-terminal -- ./arborist.sh frog_task_based.xml 8
	sleep 3
	mate-terminal -- ./sys_refl.sh
	sleep 3
	mate-terminal -- ./report.sh
	sleep 3
	mate-terminal -- ./tree_action.sh 
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


