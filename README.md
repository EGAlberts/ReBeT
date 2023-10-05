# ReBeT
Re-configuable Behavior Trees

Behavior Trees to meet Quality Requirements through Multi-armed Bandits at runtime in ROS 2.

## Dependencies 
1. [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
2. [NAV2](https://navigation.ros.org/getting_started/index.html#installation)
3. [Gazebo classic](http://classic.gazebosim.org/)

## Prerequisites
As it stands, ROS2 is only fully functional on Linux system. ROS2 Humble requires at least Ubuntu 22 (or equivalent). If you do not or are not willing to intstall Ubuntu on your device, we recommend a docker container, for example: https://hub.docker.com/r/kasmweb/ubuntu-jammy-desktop

Within the docker container you can follow the steps in the Installation section.

## Installation
1. Create a workspace folder called rebet_ws in the home directory on your device so that the it exists at ~/rebet_ws and a folder within that folder called src .

2. Clone this repository and put the rebet and rebet_msgs folders into the src folder.

3. Download release BehaviorTree.CPP-4.1.1 from https://github.com/BehaviorTree/BehaviorTree.CPP/releases/tag/4.1.1 and place it within the src folder.

4. Clone particular commit from BehaviorTree.ROS2: `git clone https://github.com/BehaviorTree/BehaviorTree.ROS2/tree/9ed45e92bee2d694789a4db6703234a1da0b8681` and also place it within the src folder. .

5. Download darknet and darknet-ros by cloning darknet-ROS recursively: `git clone --recurse-submodules -j8 https://github.com/leggedrobotics/darknet_ros.git` and also placed them within the src folder. A small change needs to be made to a one file in its code:
In darknet_ros/launch/darknet_ros.launch.py you change node_executable and node_name to just executable and name.

6. Install masced_bandits python library: `pip install masced-bandits`

7. Run colcon build in the root of your workspace folder to build all the packages.

## Run
We provide a number of scripts to easily reproduce our experiments. Simply use scripts/experiment1 and scripts/experiment2 to reproduce the two experiments from the paper. If you want do one simple run, you can use scripts/<name_of_manager>manager.sh with either UCB, Greedy, Random, or Reactive.

If you'd like to change anything about the experiments, just open up the .sh files you're running as some parameters are overriden there, and for those that aren't check the rebet_config.yaml file in the src/rebet/config folder.
