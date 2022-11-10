#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch /home/raposang/catkin_ws/src/raposang/raposang_bringup/launch/raposang_herkulex.launch &
sleep 4
roslaunch /home/raposang/catkin_ws/src/raposang/raposang_control/raposang_joy/launch/raposang_joy_node.launch joy_config:=joystick &
sleep 2
rosrun experiment_record raposang_rosbag_node.py &
