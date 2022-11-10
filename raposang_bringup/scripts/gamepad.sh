#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch raposang_joy raposang_joy_node.launch joy_config:=gamepad