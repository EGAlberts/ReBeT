#!/bin/bash
source ~/.bashrc
source ~/rebet_ws/install/setup.bash

case "$1" in
   "ID")
	if [ "$2" != "none" ] && [ "$3" != "none" ];
	then
	    echo "det rate and det_thresh given"
	    ros2 launch rebet task_servers_launch.py task_name:=$1 det_rate:=$2 det_thresh:=$3
	else
		if [ "$2" != "none" ] && [ "$3" == "none" ];
		then
		    echo "det rate given and det_thresh not given"
		    ros2 launch rebet task_servers_launch.py task_name:=$1 det_rate:=$2 det_thresh:=1
		fi
		
		if [ "$2" == "none" ] && [ "$3" != "none" ];
		then
		    echo "det rate not given and det_thresh given"
		    ros2 launch rebet task_servers_launch.py task_name:=$1 det_rate:=3 det_thresh:=$3
		fi
		
	    echo "no det rate and det_thresh given"
	    ros2 launch rebet task_servers_launch.py task_name:=$1
	fi
   ;;
   *)
   	ros2 launch rebet task_servers_launch.py task_name:=$1
   ;;
esac
