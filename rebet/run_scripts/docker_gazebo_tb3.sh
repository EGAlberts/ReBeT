#!/bin/bash
source ~/.bashrc

docker exec frog_gazebo /home/kasm-user/rebetsim_ws/gazebo_tb3.sh $1 $2
