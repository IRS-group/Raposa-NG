#! /bin/bash
### BEGIN INIT INFO
# Provides:          rc.local
# Required-Start:    $remote_fs $syslog $all
# Required-Stop:
# Default-Start:     2 3 4 5
# Default-Stop:
# Short-Description: Run /etc/rc.local if it exist
### END INIT INFO

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/catkin_ws/src:$ROS_PACKAGE_PATH
export ROS_MASTER_URI=http://raposang:11311
export ROS_IP=192.168.0.250
export ROS_HOSTNAME=raposang


echo "Running roscore daemon"
roscore
