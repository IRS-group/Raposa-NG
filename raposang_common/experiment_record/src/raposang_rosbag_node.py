#!/usr/bin/env python

import rospy
import shlex
import threading
import subprocess
from experiment_record.msg import RecData
import time
import os

isRecord = False
sp = None
thrd = None

def subpro(*arg):
    global sp
    try:
        sp = subprocess.call(arg, shell=False)
        # if sp < 0:
        #    print("Child was terminated by signal", -retcode)
        # else:
        #    print("Child returned", retcode)
    except OSError as e:
        print("Execution failed:", e)

def recordcall(data):
    global isRecord, sp, thrd

    bagname = "/home/raposang/Experiment_bags/%s_%s_%d" % (data.corp, data.cond, data.subnum)
    shellcom = "rosbag record -o %s /imu/data /joy /map /raposang/arm /raposang/attitude /raposang/bb2/left/image_color/compressed /raposang/bb2/right/image_color/compressed /raposang/depth_cam/color/image_raw/compressed /raposang/herkulex/motor_status /scan /raposang/motors/ticks /raposang/move_arm /raposang/odometry /raposang/odometry_pose /raposang/pantilt /raposang/sensors/lights /raposang/tracks /raposang/usb_cam/image_raw/compressed /slipping_broadcaster /tf __name:=bagrecord" % (
        bagname)
    args = shlex.split(shellcom)
    if data.rec and not isRecord: # to record data has to be true and isrecord false
        print("recording")
        thrd = threading.Thread(target = subpro, args= args)
        thrd.start()
        isRecord = data.rec
    elif not data.rec and isRecord: # to stop record data has to be false and isrecord true
        print("stop")
        a = shlex.split("rosnode kill /bagrecord")
        subprocess.call(a, shell=False)
        thrd.join()
        #sp.terminate()
        #sp.communicate()
        #sp = None
        isRecord = data.rec

def rec_rosbag():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('record_rosbag', anonymous=True)

    rospy.Subscriber('/rec_bag', RecData, recordcall)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rec_rosbag()