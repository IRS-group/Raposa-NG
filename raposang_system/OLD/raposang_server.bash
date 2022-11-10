#!/bin/bash

echo "Starting roscore"
gnome-terminal -x /home/raposang/catkin_ws/src/raposang/raposang_system/raposang_roscore.bash
sleep 4
echo "Running raposang nodes"
gnome-terminal -x /home/raposang/catkin_ws/src/raposang/raposang_system/raposang_init_hardware.bash
sleep 30
/home/raposang/catkin_ws/src/raposang/raposang_system/raposang_init_control.bash
echo "Done"

#rosrun map_server map_saver -f mymap

#screen -dmS roscore ~/catkin_ws/src/raposang/raposang_system/raposang_roscore.bash
#sleep 4
#screen -dmS sensor roslaunch raposang_bringup raposang_sensorsboard.launch
#screen -dmS motors roslaunch raposang_bringup raposang_motorsboard.launch
#screen -dmS herkulex roslaunch raposang_bringup raposang_herkulex.launch 
#screen -dmS imu roslaunch raposang_tf raposang_imu.launch
#sleep 1
#screen -dmS realsense roslaunch raposang_bringup raposang_rs_camera.launch
#screen -dmS usb_cam roslaunch raposang_bringup raposang_usb_cam.launch
#screen -dmS bb2 roslaunch raposang_bringup raposang_bumblebee.launch 
#screen -dmS laser rosrun urg_node urg_node 
#screen -dmS tf roslaunch raposang_tf raposang_tf.launch
#screen -dmS laser_stab rosrun raposang_laser_stabiliser laser_stabilization.py
#screen -dmS odom roslaunch raposang_odometry raposang_odometry.launch
#screen -dmS slam roslaunch raposang_slam raposang_slam.launch
#screen -dmS joy roslaunch raposang_joy raposang_joy_node.launch
#screen -dmS rosbridge roslaunch raposang_bringup raposang_rosbridge.launch 
#echo "done. view with screen -ls"

