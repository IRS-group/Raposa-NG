#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch /home/raposang/catkin_ws/src/raposang/raposang_bringup/launch/raposang_sensorsboard.launch &
roslaunch /home/raposang/catkin_ws/src/raposang/raposang_bringup/launch/raposang_imu.launch &
roslaunch /home/raposang/catkin_ws/src/raposang/raposang_bringup/launch/raposang_bumblebee.launch &
roslaunch raposang_bringup raposang_motorsboard.launch & 
rosrun urg_node urg_node &
roslaunch /home/raposang/catkin_ws/src/raposang/raposang_bringup/launch/raposang_rs_camera.launch &
roslaunch raposang_bringup raposang_herkulex.launch &
roslaunch /home/raposang/catkin_ws/src/raposang/raposang_bringup/launch/raposang_usb_cam.launch &

#bash -c "roslaunch raposang_tf raposang_tf.launch" &
#bash -c "roslaunch raposang_joy raposang_joy_node.launch joy_config:=gamepad" &
#bash -c "roslaunch raposang_joy raposang_joy_node.launch joy_config:=joystick" &
#bash -c "rosrun raposang_laser_stabiliser laser_stabilization.py" &
#bash -c "roslaunch raposang_odometry raposang_odometry.launch" &
#bash -c "roslaunch raposang_control_relay raposang_control_relay.launch" &
#bash -c "roslaunch raposang_joy joy_node.launch" &
#bash -c "roslaunch raposang_slam raposang_slam.launch" &
#bash -c "roslaunch raposang_slippage raposang_slippage.launch" &
#bash -c "roslaunch raposang_bringup raposang_rosbridge.launch" 
