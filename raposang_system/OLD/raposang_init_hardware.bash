#! /bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/catkin_ws/src:$ROS_PACKAGE_PATH
export ROS_MASTER_URI=http://raposang:11311
export ROS_IP=192.168.0.250
export ROS_HOSTNAME=raposang

echo "Starting hardware"
gnome-terminal -x roslaunch raposang_bringup raposang_sensorsboard.launch &
sleep 1
roslaunch raposang_bringup raposang_motorsboard.launch & 
sleep 2
roslaunch raposang_bringup raposang_herkulex.launch &
sleep 4
roslaunch raposang_bringup raposang_imu.launch &
sleep 3
rosrun urg_node urg_node &
sleep 4
gnome-terminal -x roslaunch raposang_bringup raposang_rs_camera.launch
sleep 2
gnome-terminal -x roslaunch raposang_bringup raposang_bumblebee.launch
sleep 2
roslaunch raposang_bringup raposang_usb_cam.launch 

