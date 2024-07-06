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
	mate-terminal -- ./darknet_auto.sh
	sleep 5
	mate-terminal -- ./start_gazebo.sh false 1
	sleep 15
	mate-terminal -- ./lighting.sh
	sleep 5
	mate-terminal -- ./nav2_map.sh 
	sleep 15
	mate-terminal -- ./arborist.sh frog_aal_arch_internal.xml 8
	sleep 25
	mate-terminal -- ./adap_sys.sh
	sleep 3
	mate-terminal -- ./report.sh
	sleep 3
	mate-terminal -- ./sys_refl.sh
	sleep 5
	mate-terminal -- ./tree_action.sh 

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
	mate-terminal -- ./darknet_auto.sh
	sleep 5
	mate-terminal -- ./start_gazebo.sh true 1
	sleep 15
	mate-terminal -- ./lighting.sh
	sleep 5
	mate-terminal -- ./nav2_map.sh 
	sleep 15
	mate-terminal -- ./arborist.sh frog_aal_arch_external.xml 8
	sleep 25
	mate-terminal -- ./adap_sys.sh
	sleep 3
	mate-terminal -- ./report.sh
	sleep 3
	mate-terminal -- ./sys_refl.sh
	sleep 5
	mate-terminal -- ./tree_action.sh 

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
	mate-terminal -- ./darknet_auto.sh
	sleep 5
	mate-terminal -- ./start_gazebo.sh true 1
	sleep 15
	mate-terminal -- ./lighting.sh
	sleep 5
	mate-terminal -- ./nav2_map.sh 
	sleep 15
	mate-terminal -- ./arborist.sh frog_aal_arch_baseline.xml 8
	sleep 25
	mate-terminal -- ./adap_sys.sh
	sleep 3
	mate-terminal -- ./report.sh
	sleep 3
	mate-terminal -- ./sys_refl.sh
	sleep 5
	mate-terminal -- ./tree_action.sh 

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
