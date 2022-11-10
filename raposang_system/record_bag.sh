#!/bin/bash
trap "rosnode kill /bagrecord" SIGINT
rosbag record -o /home/raposang/Experiment_bags/$1_$2_$3 /imu/data /joy /map /raposang/arm /raposang/attitude /raposang/bb2/left/image_color/compressed /raposang/bb2/right/image_color/compressed /raposang/depth_cam/color/image_raw/compressed /raposang/herkulex/motor_status /raposang/laser_transform /raposang/motors/ticks /raposang/move_arm /raposang/odometry /raposang/odometry_pose /raposang/pantilt /raposang/sensors/lights /raposang/tracks /raposang/usb_cam/image_raw/compressed /slipping_broadcaster /tf __name:=bagrecord

#./record_bag <team> <subj #>
