# ReBeT
Re-configurable Behavior Trees

Behavior Trees to meet Quality Requirements through Multi-armed Bandits at runtime in ROS 2.


## Dependencies 
1. [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
2. [NAV2](https://navigation.ros.org/getting_started/index.html#installation)
3. [Gazebo classic](http://classic.gazebosim.org/)

## Prerequisites
As it stands, ROS2 is only fully functional on a Linux system. ROS2 Humble requires at least Ubuntu 22 (or equivalent). If you do not or are not willing to install Ubuntu on your device, we recommend a docker container, for example: https://hub.docker.com/r/kasmweb/ubuntu-jammy-desktop

Within the docker container you can follow the steps in the Installation section.

## Installation
1. Create a workspace folder called rebet_ws in the home directory on your device so that the it exists at ~/rebet_ws and a folder within that folder called src .

2. Clone this repository and put the rebet and rebet_msgs folders into the src folder.

3. Download release BehaviorTree.CPP-4.1.1 from https://github.com/BehaviorTree/BehaviorTree.CPP/releases/tag/4.1.1 and place it within the src folder.

4. Clone particular commit from BehaviorTree.ROS2: `git clone https://github.com/BehaviorTree/BehaviorTree.ROS2/tree/9ed45e92bee2d694789a4db6703234a1da0b8681` and also place it within the src folder. .

5. Download darknet and darknet-ros by cloning darknet-ROS recursively: `git clone --recurse-submodules -j8 https://github.com/leggedrobotics/darknet_ros.git` and also placed them within the src folder. A small change needs to be made to a one file in its code:
In darknet_ros/launch/darknet_ros.launch.py you change node_executable and node_name to just executable and name.

6. Install masced_bandits python library: `pip install masced-bandits`

7. Run `colcon build` in the root of your workspace folder to build all the packages.

## Reproduction
We provide a singular script which runs all the experiments from the paper. Keep in mind that actually running them in succession take as long as 120 hours.
To run them you use the command from within the root of your workspace folder `./scripts/SEAMS24Reproduction.sh`

The result of each run get placed in a csv inside the scripts folder called rebet\_results.csv. Inquire the code inside the arborist node to see what each field is and where it comes from. If the run involves a bandit is also records its progress to a csv called bandit_report.csv in the same folder.

However, for the double-blind we could not provide the docker container we use to run the Gazebo simulator neither a fork of SUAVE.
