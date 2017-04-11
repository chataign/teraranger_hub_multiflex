# TeraRanger hub Multiflex package for ROS

This repository contains a package for [TeraRanger hub Multiflex](http://www.teraranger.com/products/teraranger-multiflex/) time of flight sensors.

## Compiling the package

The following steps show how to compile the TeraRanger hub Multiflex node with ROS.

1. Create a directory for ROS workspace: `mkdir -p ~/catkin_ws/src`
2. Initialize workspace: `cd ~/catkin_ws/src && catkin_init_workspace`
3. Clone TeraRanger hub Multiflex repository: `git clone git@github.com:Terabee/teraranger_hub_multiflex.git`
4. Compile TeraRanger hub Multiflex package: `cd ~/catkin_ws/ && catkin_make`
5. Source the files: `source devel/setup.bash`

## Running TeraRanger hub Multiflex

1. After sourcing your workspace (point 5 in compiling instructions) run the Multiflex node with: `rosrun teraranger_hub_multiflex teraranger_hub_multiflex_node`
2. To change the sensor settings use *rqt_reconfigure* while *teraranger_hub_multiflex_node* is running: `rosrun rqt_reconfigure rqt_reconfigure`

## Specifying portname during launch time

Portname can be specified during launch by specifying the _portname_. For example:
` rosrun teraranger_hub_multiflex terarger_hub_multiflex_node _portname:=/dev/ttyACM1`
