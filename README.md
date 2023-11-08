# beginner_tutorials

## Overview
This repository contains beginner tutorials in C++ for a publisher and subscriber node in ROS2 humble for custom string message

## Dependencies
* ROS 2 Humble
* Ubuntu 22.04

## Build Instructions
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/NehaMadhekar09/beginner_tutorials.git

cd ..

rosdep install -i --from-path src --rosdistro humble -y

colcon build 

. install/setup.bash

```
## Run instructions
1. Go to directory
```
cd ros2_ws
```
2. Source the workspace
```
. install/setup.bash
```
3. To run talker and listener nodes
   
In terminal 1 
```
ros2 run First_Publisher_Subscriber talker
```
Open another terminal 2
```
. install/setup.bash
```
```
ros2 run First_Publisher_Subscriber listener
```
