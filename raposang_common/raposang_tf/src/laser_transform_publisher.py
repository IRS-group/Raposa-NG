#!/usr/bin/env python
import math
import rospy
import numpy
from transforms3d import euler, quaternions as qua #https://matthew-brett.github.io/transforms3d/index.html

# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu

# ------------------------------------------------------------
#                         GLOBALS
# ------------------------------------------------------------
transfStamped = 0

# ------------------------------------------------------------
#                         FUNCTIONS
# ------------------------------------------------------------
def getIMU(imu_msg):
    global transfStamped
    q_imu = (
        imu_msg.orientation.w,
        imu_msg.orientation.x,
        imu_msg.orientation.y,
        imu_msg.orientation.z)

    transf_q = (transfStamped.transform.rotation.w,
                transfStamped.transform.rotation.x,
                transfStamped.transform.rotation.y,
                transfStamped.transform.rotation.z)

    # transf_laser = qua.qinverse(qua.qmult(q_imu, transf_q))
    # transf_laser = qua.qmult(qua.qinverse(q_imu), transf_q)
    transf_laser = qua.qmult(q_imu, transf_q)

    yaw, pitch, roll = euler.quat2euler(transf_laser, 'rzyx')

    rospy.loginfo(
        'Yaw:%d Pitch:%d Roll:%d' % (int(math.degrees(yaw)), int(math.degrees(pitch)), int(math.degrees(roll))))

    msg = geometry_msgs.msg.QuaternionStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.quaternion = geometry_msgs.msg.Quaternion(transf_laser[1], transf_laser[2], transf_laser[3], transf_laser[0])

    transf_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('laser_transform')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # try:
    transfStamped = tfBuffer.lookup_transform('imu', 'base_link', rospy.Time(), rospy.Duration(3))
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #   pass

    transf_pub = rospy.Publisher('/raposang/laser_transform', geometry_msgs.msg.QuaternionStamped, queue_size=1)
    rospy.Subscriber("/imu/data", Imu, getIMU)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        rate.sleep()
