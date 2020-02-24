# ROS Turtlebot Walker - ENPM690
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Project Overview

This package implements a Obstacle Avoiding 2 wheeled robot in Gazebo.

## Dependencies
The project has the following dependencies.

1. ROS Kinetic ([Installation](http://wiki.ros.org/kinetic/Installation))
2. catkin ([Installation](http://wiki.ros.org/catkin#Installing_catkin))
3. Turtlebot Packages
```
sudo apt-get install ros-kinetic-turtlebot-*
```

## Build steps
 
- To build the given project, create and build the catkin workspace by following the given steps:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
cd src/
git clone --recursive https://github.com/rohansingh42/walker.git
cd ~/catkin_ws/
catkin_make
```

- NOTE: For running command from each new terminal, source the devel/setup.bash file in every new terminal before executing any ros command, or add the following line to your _~/.bashrc_ once
```
source ~/catkin_ws/devel/setup.bash
```

## Running the demo

- To run the simulation demo using the launch file (after sourcing _devel/setup.bash_ according to above note), in a new terminal type,
```
roslaunch turtlebot_walker walkerDemo.launch
```
