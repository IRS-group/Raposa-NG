#!/usr/bin/env python

import rospy
import numpy
import rosbag
import message_filters
from sensor_msgs.msg import Imu, Joy, LaserScan, CompressedImage
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int16, Int32, Float32, Bool
from idmind_herkulex.msg import Motors
from idmind_motorsboard.msg import WheelsMB
from raposang_control_relay.msg import Tracks
from experiment_record.msg import RecData
import time

joy = "/joy" 
map = "/map"
laser = "/scan"  
imu = "/imu/data" 
attitude = "/raposang/attitude"
tf = "/tf"
odom = "/raposang/odometry" 
odom_pose = "/raposang/odometry_pose" 
arm = "/raposang/arm" 
arm_input = "/raposang/move_arm" 
motors = "/raposang/motors/ticks" 
tracks_input = "/raposang/tracks" 
slipping = "/slipping_broadcaster"
herkulex = "/raposang/herkulex/motor_status" 
pan_tilt_input = "/raposang/pantilt" 
bb2_left = "/raposang/bb2/left/image_color/compressed" 
bb2_right = "/raposang/bb2/right/image_color/compressed" 
depth = "/raposang/depth_cam/color/image_raw/compressed"
usb_cam = "/raposang/usb_cam/image_raw/compressed"
lights = "/raposang/sensors/lights" 

isRecord = False
isFirst = True
bagname = 0
#bag = 0

def writetobag(mode, j=0, m=0, l=0, i=0, att=0, t=0, o=0, op=0, a=0, ai=0, mt=0, tr=0, s=0, h=0, pt=0, bl=0, br=0, d=0, u=0, lg=0):
    #global bag
    bag = rosbag.Bag(bagname, mode)
    try:
        if j != 0:
            bag.write(joy, j)
        if m != 0:
            bag.write(map, m)
        if l != 0:
            bag.write(laser, l)
        if i != 0:
            bag.write(imu, i)
        if att != 0:
            bag.write(attitude, att)
        if t != 0:
            bag.write(tf, t)
        if o != 0:
            bag.write(odom, o)
        if op != 0:
            bag.write(odom_pose, op)
        if a != 0:
            bag.write(arm, a)
        if ai != 0:
            bag.write(arm_input, ai)
        if mt != 0:
            bag.write(motors, mt)
        if tr != 0:
            bag.write(tracks_input, tr)
        if s != 0:
            bag.write(slipping, s)
        if h != 0:
            bag.write(herkulex, h)
        if pt != 0:
            bag.write(pan_tilt_input, pt)
        if bl != 0:
            bag.write(bb2_left, bl)
        if br != 0:
            bag.write(bb2_right, br)
        if d != 0:
            bag.write(depth, d)
        if u != 0:
            bag.write(usb_cam, u)
        if lg != 0:
            bag.write(lights, lg)
    # except e:
    #    rospy.logerr("bag variable is not initialized, %s", e)
    finally:
        bag.close()

def recordcall(data):
    global isRecord, bagname, isFirst

    if data.rec and not isRecord: # to record data has to be true and isrecord false
        bagname = "/home/raposang/Experiment_bags/%s_%s_%d.bag"%(data.corp, data.cond, data.subnum)
        isRecord = data.rec
    elif not data.rec and isRecord: # to stop record data has to be false and isrecord true
        #bag.close()
        isRecord = data.rec
        isFirst = True


def motorscallback(ai, mt, pt):
    global isFirst

    print('Motors')
    if isRecord:
        if isFirst:
            writetobag('w', 0, 0, 0, 0, 0, 0, 0, 0, 0, ai, mt, 0, 0, 0, pt, 0, 0, 0, 0, 0)
            isFirst = False
        else:
            writetobag('a', 0, 0, 0, 0, 0, 0, 0, 0, 0, ai, mt, 0, 0, 0, pt, 0, 0, 0, 0, 0)

def videocallback(l, i, t, o, op, tr, bl, br, d, u, lg):
    global isFirst

    now = rospy.Time.now()
    rospy.loginfo("Video %i %i", now.secs, now.nsecs)
    if isRecord:
        if isFirst:
            writetobag('w', 0, 0, l, i, 0, t, o, op, 0, 0, 0, tr, 0, 0, 0, bl, br, d, u, lg)
            isFirst = False
        else:
            writetobag('a', 0, 0, l, i, 0, t, o, op, 0, 0, 0, tr, 0, 0, 0, bl, br, d, u, lg)


def rec_rosbag():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('record_rosbag', anonymous=True)

    joy_sub = message_filters.Subscriber(joy, Joy)
    map_sub = message_filters.Subscriber(map, OccupancyGrid)
    laser_sub = message_filters.Subscriber(laser, LaserScan)
    imu_sub = message_filters.Subscriber(imu, Imu)
    attitude_sub = message_filters.Subscriber(attitude, Vector3) #no header
    tf_sub = message_filters.Subscriber(tf, TFMessage)
    odom_sub = message_filters.Subscriber(odom, WheelsMB)
    odom_pose_sub = message_filters.Subscriber(odom_pose, Odometry)
    arm_sub = message_filters.Subscriber(arm, Int32) # no header
    arm_input_sub = message_filters.Subscriber(arm_input, Float32) #IS not always pub, no header
    motors_sub = message_filters.Subscriber(motors, WheelsMB) #IS not always pub
    tracks_input_sub = message_filters.Subscriber(tracks_input, Tracks)
    slipping_sub = message_filters.Subscriber(slipping, Int16) # no header
    herkulex_sub = message_filters.Subscriber(herkulex, Motors) # no header
    pan_tilt_input_sub = message_filters.Subscriber(pan_tilt_input, Vector3) #IS not always pub, no header
    bb2_left_sub = message_filters.Subscriber(bb2_left, CompressedImage)
    bb2_right_sub = message_filters.Subscriber(bb2_right, CompressedImage)
    depth_sub = message_filters.Subscriber(depth, CompressedImage)
    usb_cam_sub = message_filters.Subscriber(usb_cam, CompressedImage)
    lights_sub = message_filters.Subscriber(lights, Bool) # no header

    #joy_sub, map_sub,  attitude_sub, arm_sub, slipping_sub, herkulex_sub,
    tsV = message_filters.ApproximateTimeSynchronizer([laser_sub, imu_sub, tf_sub, odom_sub, odom_pose_sub, tracks_input_sub, bb2_left_sub, bb2_right_sub, depth_sub, usb_cam_sub, lights_sub], 10, 0.1, allow_headerless=True)
    tsV.registerCallback(videocallback)

    tsM = message_filters.ApproximateTimeSynchronizer([arm_input_sub, motors_sub, pan_tilt_input_sub], 10, 0.1, allow_headerless=True)
    tsM.registerCallback(motorscallback)

    rospy.Subscriber('/rec_bag', RecData, recordcall)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rec_rosbag()
