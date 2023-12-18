#!/bin/bash

trap ctrl_c INT

function ctrl_c() {
        echo "Cleaning up.."
	./killall.sh
        echo "Shutting down"
        exit 0
}


cd scripts


for EXPNUM in {1..10}
do
	gnome-terminal -- ./darknet.sh
	sleep 15
	gnome-terminal -- ./frontier_service.sh
	sleep 3
	gnome-terminal -- ./docker_gazebo_tb3.sh false 1
	sleep 15
	gnome-terminal -- ./nav2.sh 
	sleep 15
	gnome-terminal -- ./arborist.sh test frog_online.xml 0 0
	sleep 3
	gnome-terminal -- ./sys_refl.sh
	sleep 3
	gnome-terminal -- ./adap_sys.sh
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

for EXPNUM in {1..10}
do
	gnome-terminal -- ./darknet.sh
	sleep 15
	gnome-terminal -- ./frontier_service.sh
	sleep 3
	gnome-terminal -- ./docker_gazebo_tb3.sh true 1
	sleep 15
	gnome-terminal -- ./nav2.sh 
	sleep 15
	gnome-terminal -- ./arborist.sh test frog.xml 0 0
	sleep 3
	gnome-terminal -- ./sys_refl.sh
	sleep 3
	gnome-terminal -- ./adap_sys.sh
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

for EXPNUM in {1..10}
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


for EXPNUM in {1..20}
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
