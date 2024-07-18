# ReBeT
Reconfiguration with Behavior Trees.

This is a library which adds two new types of BT nodes to BT.CPP/BT.ROS2: QRDecorators and AdaptDecorators.
The former makes it possible to explicitly represent quality requirements, and reason against these with BT logic, or logic specific in AdaptDecorators.
The latter provides a hook for adapting ROS2 nodes, changing their parameters, lifecycle states, or topics. 

We have written a paper to be published at ACSOS 2024 (https://2024.acsos.org/) about this work, please see the preprint.pdf in the root of this repository.

##Installation

0. This package is made to work with ROS2 Humble, so ensure you have this installed on your system.
1. Ensure you have BT.CPP (https://github.com/BehaviorTree/BehaviorTree.CPP), and BT.ROS2 (https://github.com/BehaviorTree/BehaviorTree.ROS2) installed in a ROS2 workspace.
2. This work makes use of my other work AAL(https://github.com/EGAlberts/AAL-ROS2) which provides an architectural adaptation layer for ROS2. Simply clone AAL into the src folder in a ROS2 workspace.
3. Clone this repository into the src folder of your ROS2 workspace.
4. Use colcon build in the root of your workspace to build all the packages.
5. To get a quick idea check out rebet_samples. The launch file in there can be used alongisde echo-ing the ROS2 topic sample_topic to get an idea of what ReBeT does.


NOTE: This READMe is a bit WIP, and is subject to change (for the better) in the coming weeks.
   
