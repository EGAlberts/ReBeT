#!/bin/bash

trap ctrl_c INT

function ctrl_c() {
        echo "Cleaning up.."
	./killall.sh
        echo "Shutting down"
        exit 0
}


cd scripts


for EXPNUM in {1..25}
do
	xfce4-terminal -e ./darknet_auto.sh
	sleep 5
	xfce4-terminal -e "./start_gazebo.sh true 1"
	sleep 15
	xfce4-terminal -e ./lighting.sh
	sleep 5
	xfce4-terminal -e ./nav2_map.sh 
	sleep 15
	xfce4-terminal -e "./arborist.sh frog_aal_arch_baseline.xml 8"
	sleep 25
	xfce4-terminal -e ./adap_sys.sh
	sleep 3
	xfce4-terminal -e ./report.sh
	sleep 3
	xfce4-terminal -e ./sys_refl.sh
	sleep 5
	xfce4-terminal -e ./tree_action.sh 

	SECONDS=0 
	while [ ! -f ~/rebet_ws/scripts/mission.done -a $SECONDS -lt 400 ]
	do
	   echo "waiting for mission to be done" $EXP             
	sleep 10 #sustainability!
	done
	echo "mission done"
	rm ~/rebet_ws/scripts/mission.done
	./killall.sh	
done

for EXPNUM in {1..25}
do
	xfce4-terminal -e ./darknet_auto.sh
	sleep 5
	xfce4-terminal -e "./start_gazebo.sh true 1"
	sleep 15
	xfce4-terminal -e ./lighting.sh
	sleep 5
	xfce4-terminal -e ./nav2_map.sh 
	sleep 15
	xfce4-terminal -e "./arborist.sh frog_aal_arch_internal.xml 8"
	sleep 25
	xfce4-terminal -e ./adap_sys.sh
	sleep 3
	xfce4-terminal -e ./report.sh
	sleep 3
	xfce4-terminal -e ./sys_refl.sh
	sleep 5
	xfce4-terminal -e ./tree_action.sh 

	SECONDS=0 
	while [ ! -f ~/rebet_ws/scripts/mission.done -a $SECONDS -lt 400 ]
	do
	   echo "waiting for mission to be done" $EXP             
	sleep 10 #sustainability!
	done
	echo "mission done"
	rm ~/rebet_ws/scripts/mission.done
	./killall.sh	
done

for EXPNUM in {1..25}
do
	xfce4-terminal -e ./darknet_auto.sh
	sleep 5
	xfce4-terminal -e "./start_gazebo.sh true 1"
	sleep 15
	xfce4-terminal -e ./lighting.sh
	sleep 5
	xfce4-terminal -e ./nav2_map.sh 
	sleep 15
	xfce4-terminal -e "./arborist.sh frog_aal_arch_external.xml 8"
	sleep 25
	xfce4-terminal -e ./adap_sys.sh
	sleep 3
	xfce4-terminal -e ./report.sh
	sleep 3
	xfce4-terminal -e ./sys_refl.sh
	sleep 5
	xfce4-terminal -e ./tree_action.sh 

	SECONDS=0 
	while [ ! -f ~/rebet_ws/scripts/mission.done -a $SECONDS -lt 400 ]
	do
	   echo "waiting for mission to be done" $EXP             
	sleep 10 #sustainability!
	done
	echo "mission done"
	rm ~/rebet_ws/scripts/mission.done
	./killall.sh	
done

