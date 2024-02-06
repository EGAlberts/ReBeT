#!/bin/bash

trap ctrl_c INT

function ctrl_c() {
        echo "Cleaning up.."
	./killall.sh
        echo "Shutting down"
        exit 0
}


cd scripts


for EXPNUM in {1..4}
do
	gnome-terminal -- ./launch_suave_sim.sh
	sleep 60
	gnome-terminal -- ./arborist.sh test suave_offline.xml 0 0
	sleep 3
	gnome-terminal -- ./sys_refl.sh
	sleep 3
	gnome-terminal -- ./adap_sys.sh
	sleep 3
	gnome-terminal -- ./tree_action.sh 
	SECONDS=0 
	while [ ! -f ~/rebet_ws/scripts/mission.done -a $SECONDS -lt 600 ]
	do
	   echo "waiting for mission to be done" $EXP             
	sleep 10 #sustainability!
	done
	echo "mission done"
	rm ~/rebet_ws/scripts/mission.done
	./killall.sh	
done


