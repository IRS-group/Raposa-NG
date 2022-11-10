#! /bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/catkin_ws/src:$ROS_PACKAGE_PATH
export ROS_MASTER_URI=http://raposang:11311
export ROS_IP=192.168.0.250
export ROS_HOSTNAME=raposang

echo "Starting Control"
roslaunch raposang_tf raposang_tf.launch &
sleep 2
rosrun raposang_laser_stabiliser laser_stabilization.py &
sleep 1
roslaunch raposang_odometry raposang_odometry.launch &
sleep 2
gnome-terminal -x roslaunch raposang_slam raposang_slam.launch
roslaunch raposang_control_relay raposang_control_relay.launch &
roslaunch raposang_joy joy_node.launch &
gnome-terminal -x roslaunch raposang_joy raposang_joy_node.launch joy_config:=gamepad
sleep 4
gnome-terminal -x roslaunch raposang_joy raposang_joy_node.launch joy_config:=joystick
roslaunch raposang_slippage raposang_slippage.launch &
gnome-terminal -x roslaunch raposang_bringup raposang_rosbridge.launch 
